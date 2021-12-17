# Secure Boot를 활용한 NXP 드론의 시스템 보안성 연구/개발
![Screenshot from 2021-12-17 13-11-32](https://user-images.githubusercontent.com/55688999/146487762-f212140f-d16b-44a8-b729-fb8f67b0b41d.png) </br>
프로젝트에서 활용한 NXP 호버게임스의 PX4 지원 보드(NXP-RDDRONE-FMUK66) </br>
해당 NXP 보드는 SE를 포함하고 있지만 부팅과정에서의 무결성 검증 과정이 부재하여 Secure Boot를 통해 시스템 보안성 강화 연구/개발을 진행함 </br>

## PX4-Autopilot-Firmware
- https://github.com/PX4/PX4-Autopilot </br>

## PX4-Bootloader
- https://github.com/PX4/PX4-Bootloader </br>

## ECDSA 
![Screenshot from 2021-12-17 13-28-55](https://user-images.githubusercontent.com/55688999/146489093-83643a94-2412-4380-ae40-667270add662.png) </br>![Screenshot from 2021-12-17 13-30-29](https://user-images.githubusercontent.com/55688999/146489211-14c0ac63-9674-4195-8d6f-aa478cc80ca3.png) </br> 
![Screenshot from 2021-12-17 13-30-35](https://user-images.githubusercontent.com/55688999/146489235-8553af2f-878c-45ae-86bd-19c07723122f.png) </br>
ECDSA 전자서명 생성은 로컬 컴퓨터의 Visual Studio 환경에서, 검증은 nxpexpresso 환경에서 실행하여 디버깅 결과를 확인함
