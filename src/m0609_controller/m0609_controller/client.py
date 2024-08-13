import subprocess
import os
from ament_index_python.packages import get_package_share_directory

package_name = 'm0609_controller'

def main():
    # 현재 작업 디렉토리를 가져옴
    package_path = get_package_share_directory(package_name)

    # 실행 파일의 경로를 지정
    exe_path = os.path.join(package_path, "include", "test")

    # 실행 파일 실행
    try:
        result = subprocess.run([exe_path], check=True)
        print("실행 결과:", result)
    except subprocess.CalledProcessError as e:
        print(f"실행 중 오류 발생: {e}")
    except FileNotFoundError:
        print("파일을 찾을 수 없습니다:", exe_path)
        

if __name__ == '__main__':
    main()

