import submodules.diffusion_policy as dp
import torch
import rclpy
import torch.nn as nn
import numpy as np
from tqdm.auto import tqdm
from skvideo.io import vwrite
import cv2
import time
from submodules.aruco import TShapeDetector
import subprocess
import os
import gdown
from ament_index_python.packages import get_package_share_directory
from m0609_controller.trajectoryplan_client import TrajectoryPlanClient


def main():
    rclpy.init()
    vision_encoder = dp.get_resnet('resnet18')

    vision_encoder = dp.replace_bn_with_gn(vision_encoder)


    package_path = get_package_share_directory('pusht')
    
    dataset_path = package_path+"/include/pusht_cchi_v7_replay.zarr.zip"
    if not os.path.isfile(dataset_path):
        dataset_path = package_path+"/../../../../src/pusht/include/pusht_cchi_v7_replay.zarr.zip"
        id = "1KY1InLurpMvJDRb14L9NlXT_fEsCvVUq&confirm=t"
        gdown.download(id=id, output=dataset_path, quiet=False)
        print("rebuild ros workspace")
        return
        
    pred_horizon=16
    obs_horizon=2
    action_horizon=8

    dataset = dp.PushTImageDataset(
        dataset_path=dataset_path,
        pred_horizon=pred_horizon,       # 16
        obs_horizon=obs_horizon,         #  2
        action_horizon=action_horizon    #  8
    )
    stats = dataset.stats

    vision_feature_dim =512
    lowdim_obs_dim=2
    obs_dim = vision_feature_dim + lowdim_obs_dim
    action_dim =2

    noise_pred_net=dp.ConditionalUnet1D(
        input_dim=action_dim,
        global_cond_dim=obs_dim*obs_horizon
    )

    nets = nn.ModuleDict({
        'vision_encoder': vision_encoder,
        'noise_pred_net': noise_pred_net
    })

    num_diffusion_iters=100
    noise_scheduler=dp.DDPMScheduler(
        num_train_timesteps=num_diffusion_iters,
        beta_schedule='squaredcos_cap_v2',
        clip_sample=True,
        prediction_type='epsilon'
    )

    device =torch.device('cpu')
    _=nets.to(device)

    ckpt_path = package_path+"/include/pusht_vision_100ep.ckpt"
    print(ckpt_path)
    if not os.path.isfile(ckpt_path):
        id = "1XKpfNSlwYMGaF5CncoFaLKCDTWoLAHf1&confirm=t"
        ckpt_path = package_path+"/../../../../src/pusht/include/pusht_vision_100ep.ckpt"
        gdown.download(id=id, output=ckpt_path, quiet=False)
        print("rebuild ros workspace")
        return
    
    state_dict = torch.load(ckpt_path, map_location='cpu')
    ema_nets = nets
    ema_nets.load_state_dict(state_dict)
    print('Pretrained weights loaded.')


    max_steps=40
    env=dp.PushTImageEnv()
    t_detector=TShapeDetector(0)
    client_service=TrajectoryPlanClient()

    is_end=False
    agent_start_pos=[50,50]
    
    while not is_end:
        
        while(True):
            global cam_img
            angle,centor=t_detector.detect_t_shape()
            if angle==None or centor==None:
                print("anyting detect")
                time.sleep(2.0)
                continue
            cam_img=t_detector.frame
            break


        env.reset_to_state=np.array([agent_start_pos[0],agent_start_pos[1], centor[0],centor[1], angle])
        obs,info =env.reset()

        obs_deque =dp.collections.deque(
            [obs]*obs_horizon,maxlen=obs_horizon)
        imgs=[env.render(mode='rgb_array')]
        rewards=list()
        done=False
        step_idx=0

        action_list=[]

        with tqdm(total=max_steps,desc="Eval PushTImageEnv")as pbar:
            while not done:
                B=1
                
                images = np.stack([x['image'] for x in obs_deque])
                agent_poses = np.stack([x['agent_pos']for x in obs_deque])

                nagent_poses= dp.normalize_data(agent_poses,stats=stats['agent_pos'])
                nimages=images
                
                nimages=torch.from_numpy(nimages).to(device,dtype=torch.float32)
                nagent_poses=torch.from_numpy(nagent_poses).to(device,dtype=torch.float32)
                
                with torch.no_grad():
                    image_features = ema_nets['vision_encoder'](nimages)
                    # (2,512)

                    # concat with low-dim observations
                    obs_features = torch.cat([image_features, nagent_poses], dim=-1)

                    # reshape observation to (B,obs_horizon*obs_dim)
                    obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)

                    # initialize action from Guassian noise
                    noisy_action = torch.randn(
                        (B, pred_horizon, action_dim), device=device)
                    naction = noisy_action

                    # init scheduler
                    noise_scheduler.set_timesteps(num_diffusion_iters)

                    for k in noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = ema_nets['noise_pred_net'](
                            sample=naction,
                            timestep=k,
                            global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        naction = noise_scheduler.step(
                            model_output=noise_pred,
                            timestep=k,
                            sample=naction
                        ).prev_sample

                # unnormalize action
                naction = naction.detach().to('cpu').numpy()
                # (B, pred_horizon, action_dim)
                naction = naction[0]
                action_pred = dp.unnormalize_data(naction, stats=stats['action'])

                # only take action_horizon number of actions
                start = obs_horizon - 1
                end = start + action_horizon
                action =  action_pred[start:end,:]
                # (action_horizon, action_dim)

                # execute action_horizon number of steps
                # without replanning
                for act in action:
                    action_list.append(act)
                
                ##to show trajectory
                # predit_imge=imgs[-1]
                # predit_imge = cv2.resize(predit_imge, dsize=(512, 512), interpolation=cv2.INTER_AREA)
                # for pos in action:
                #     cv2.circle(predit_imge,(int(pos[0]),int(pos[1])),5,(0,0,255),-1)
                # cv2.imshow("pimg",predit_imge)
                # cv2.waitKey(0)

                
                
                for i in range(len(action)):
                    # stepping env
                    obs, reward, done, _, info = env.step(action[i])
                    # save observations
                    obs_deque.append(obs)
                    # and reward/vis
                    rewards.append(reward)
                    imgs.append(env.render(mode='rgb_array'))
                    # cv2.imshow("imgs",imgs[-1])
                    # cv2.waitKey(0)
                    # # update progress bar
                    step_idx += 1
                    pbar.update(1)
                    pbar.set_postfix(reward=reward)
                    # print(step_idx,"steps")
                    if step_idx > max_steps:
                        done = True
                    if done:
                        
                        break


        # print(action_list)

        predit_imge=cam_img
        for pos in action_list:
            cv2.circle(predit_imge,(int(pos[0]),int(pos[1])),5,(0,0,255),-1)
        cv2.imshow("pimg",predit_imge)
        cv2.waitKey(0)
        # print out the maximum target coverage
        print('Score: ', max(rewards))
        if rewards[-1]>0.9:
            is_end=True
        client_service.send_request(action_list)
        agent_start_pos=[action_list[-2][0],action_list[-2][1]]
        action_list.clear()
        

        # visualize
        # from IPython.display import Video
        # vwrite('~/vis.mp4', imgs)


if __name__ == '__main__':
    main()