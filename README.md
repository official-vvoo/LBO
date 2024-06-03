![LBO Intro](./Docs/images/LBO%20Intro.jpeg)
## 프로젝트 요약

---
|제목|내용|
|:---:|:---|
|프로젝트 명|LBO (엘보)|
|프로젝트 주제|시각장애인 실내 안내 로봇 대여 서비스|
|프로젝트 기간|2024.04.09. ~ 2024.05.17.|
|주최|삼성 청년 SW 아카데미|
|팀 구성| 정현우(팀장), 김승배, 김영래, 김태환, 박건률, 정원영|
|활용 도구|<img src="https://img.shields.io/badge/gitlab-%23FC6D26?style=for-the-badge&logo=gitlab&logoColor=white"> <img src="https://img.shields.io/badge/jira-%230052CC?style=for-the-badge&logo=jira&logoColor=white"> <img src="https://img.shields.io/badge/gerrit-%23EEEEEE?style=for-the-badge&logo=gerrit&logoColor=black"> <img src="https://img.shields.io/badge/notion-%23000000?style=for-the-badge&logo=notion&logoColor=white">|

## 결과물 및 기술 스택
---
||안내 로봇 (Robot)|안내 손잡이 (Handle)|
|:---:|:---:|:---:|
|Output|![Image_Robot](./Docs/images/안내%20로봇.jpg)|![Image_Handle](./Docs/images/안내%20손잡이.jpg)|
|H/W|<img src="https://img.shields.io/badge/jetson_nano-%2376B900?style=for-the-badge&logo=nvidia&logoColor=white">|<img src="https://img.shields.io/badge/esp32-%23E7352C?style=for-the-badge&logo=espressif&logoColor=white">|
|Language|<img src="https://img.shields.io/badge/python-%233776AB?style=for-the-badge&logo=python&logoColor=white">|<img src="https://img.shields.io/badge/c%2B%2B-%2300599C?style=for-the-badge&logo=cplusplus&logoColor=white">|
|Software|<img src="https://img.shields.io/badge/Ubuntu--20.04-%23E95420?style=for-the-badge&logo=ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/ROS2--Foxy-%2322314E?style=for-the-badge&logo=ROS"> <img src="https://img.shields.io/badge/opencv-%235C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/tensorflow_lite-%23FF6F00?style=for-the-badge&logo=tensorflow&logoColor=white"> <img src="https://img.shields.io/badge/gtts-%234285F4?style=for-the-badge&logo=googlecloud&logoColor=white">  <img src="https://img.shields.io/badge/Qt-41CD52?style=for-the-badge&logo=qt&logoColor=white">|<img src="./Docs/images/Badge_FreeRTOS.svg" alt="freeRTOS" style="max-width: 100%;">|

## 주요 문서
|No.|Doc.|Desc.|Notes|
|:---:|:---|:---|:---|
|1|[프로젝트 보고서](./Docs/Project%20Report.md)|프로젝트 전반적인 내용 및 결과 기록| <img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">|
|2|[안내 로봇 문서](./Robot/README.md)|안내 로봇 사용법 및 개발 내용 정리| <img src="https://img.shields.io/badge/작성 예정-%23E7352C?style=for-the-badge&logoColor=white">|
|3|[안내 손잡이 문서](./Handle/README.md)|안내 손잡이 사용법 및 개발 내용 정리| <img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">|
|4|[Setup 프로그램 문서](./Qt/README.md)|대여자 초기 세팅 메뉴얼| <img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">|
<!-- 
<img src="https://img.shields.io/badge/작성 예정-%23E7352C?style=for-the-badge&logoColor=white">
<img src="https://img.shields.io/badge/현재 작성 중-%23FF6F00?style=for-the-badge&logoColor=white">
<img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">
 -->
## 문서 이력
|Ver.|Desc.|Author|Date|Notes|
|:---:|:---|:---:|:---:|:---|
|0.1|README.md 생성|정현우|2024-04-09|git 생성|
|0.2|주제 선정, 배경 및 기획 추가|정현우|2024-04-22|중간 발표|
|0.3|README.md 구조 변경|정현우|2024-05-22|프로젝트 종료|
|0.4|기술 스택 및 문서 추가|정현우|2024-05-27||