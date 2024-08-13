import cv2
import numpy as np
import cv2.aruco as aruco

class TShapeDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        

    def get_warped_image(self, frame, points):
        rect = np.array(points, dtype="float32")

        # 원하는 크기 (전체 화면 크기)
        maxWidth = 512
        maxHeight = 512

        # 목표 사각형 좌표
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")

        # 원근 변환 행렬
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(frame, M, (maxWidth, maxHeight))
        return warped

    def process_frame(self, frame):
        # ArUco 마커 탐지
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
        parameters = aruco.DetectorParameters_create()
        frame=cv2.flip(frame,-1)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        points = []

        if ids is not None:
            # 각 마커의 중심 좌표를 계산
            for corner in corners:
                cX = int(np.mean(corner[0][:, 0]))
                cY = int(np.mean(corner[0][:, 1]))
                points.append((cX, cY))

            # 마커가 4개 이상 감지되면 사각형 그리기
            if len(points) >= 4:
                # y 좌표를 기준으로 정렬합니다.
                points = np.array(points)
                points = points[np.argsort(points[:, 1])]

                # 상단과 하단에 해당하는 두 점씩 나눕니다.
                top_points = points[:2]
                bottom_points = points[2:]

                # 상단 점들을 x 좌표 기준으로 정렬합니다.
                top_left = top_points[np.argmin(top_points[:, 0])]
                top_right = top_points[np.argmax(top_points[:, 0])]

                # 하단 점들을 x 좌표 기준으로 정렬합니다.
                bottom_left = bottom_points[np.argmin(bottom_points[:, 0])]
                bottom_right = bottom_points[np.argmax(bottom_points[:, 0])]

                # 점들을 순서대로 정렬합니다.
                ordered_points = np.array([top_left, top_right, bottom_right, bottom_left], dtype='float32')

                # 사각형 그리기
                for point in ordered_points:
                    point = tuple(map(int, point))  # 정수형 튜플로 변환
                    cv2.circle(frame, point, 5, (0, 255, 0), -1)

                # 사각형 선 그리기
                ordered_points_int = np.array(ordered_points, dtype=np.int32)
                cv2.polylines(frame, [np.array(ordered_points_int)], isClosed=True, color=(0, 255, 0), thickness=2)

                # 원근 변환
                warped = self.get_warped_image(frame, ordered_points)
                return frame, warped

        return frame, frame

    def find_mid_point(self, p1, p2):
        return (int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2))

    def distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def calculate_angle(self, p1, p2):
        # 기울기 계산
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # 각도 계산
        angle = np.arctan2(dy, dx)

        # -π에서 π 사이로 각도 조정
        if angle < -np.pi:
            angle += 2 * np.pi
        if angle > np.pi:
            angle -= 2 * np.pi

        return -np.pi + angle

    def detect_t_shape(self):
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("카메라로부터 프레임을 읽을 수 없습니다.")
                break

            # 프레임 처리
            original_frame, processed_frame = self.process_frame(frame)

            blurred = cv2.GaussianBlur(processed_frame, (5, 5), 0)

            # HSV 색상 공간으로 변환
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 빨간색 범위 정의
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 | mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            try:
                for contour in contours:
                    # T자 모형의 윤곽선만 선택 (여기서 조건을 추가할 수 있음)
                    if cv2.contourArea(contour) > 100:  # 면적 기준

                        # 윤곽선 근사화
                        epsilon = 0.01 * cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, epsilon, True)

                        if len(approx) < 4:
                            continue  # 꼭지점이 4개 미만이면 스킵

                        # 변의 길이를 계산하고 정렬
                        edges = []
                        for i in range(len(approx)):
                            p1 = approx[i][0]
                            p2 = approx[(i + 1) % len(approx)][0]
                            edges.append((self.distance(p1, p2), p1, p2))

                        edges.sort(key=lambda x: x[0], reverse=True)  # 길이를 기준으로 내림차순 정렬

                        # 4번과 5번 변의 중심점을 계산하고, 그 중간점을 중심점으로 설정
                        mid_point4 = self.find_mid_point(edges[3][1], edges[3][2])
                        mid_point5 = self.find_mid_point(edges[4][1], edges[4][2])

                        center_point = self.find_mid_point(mid_point4, mid_point5)

                        # 첫번째로 긴 변으로 각도값 계산
                        angle = self.calculate_angle(edges[0][1], edges[0][2])

                        # 중심점 표시
                        cv2.circle(processed_frame, center_point, 5, (255, 0, 0), -1)

                        # 윤곽선 그리기
                        cv2.drawContours(processed_frame, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(processed_frame, f'Center: ({center_point[0]}, {center_point[1]})', 
                                    (center_point[0] + 10, center_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.5, (255, 255, 255), 1)
                        # cv2.putText(processed_frame, f'Angle: {angle:.2f}',
                        #             (center_point[0] - 50, center_point[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 
                        #             0.5, (255, 255, 255), 1)

                        # 결과 프레임 보기
                        cv2.imshow('Processed Frame', processed_frame)

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.cap.release()
                            cv2.destroyAllWindows()
                            return angle, center_point
            except:
                continue

            # cv2.imshow('Original Frame', original_frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

        self.cap.release()
        cv2.destroyAllWindows()
        return None, None

# 사용 예시 (다른 파일에서 사용할 때)
if __name__ == "__main__":
    detector = TShapeDetector(0)
    angle, center = detector.detect_t_shape()
    print(f'Angle: {angle}, Center: {center}')
