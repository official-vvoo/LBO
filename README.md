# LBO(엘보)

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
|Software|<img src="https://img.shields.io/badge/Ubuntu--20.04-%23E95420?style=for-the-badge&logo=ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/ROS2--Foxy-%2322314E?style=for-the-badge&logo=ROS"> <img src="https://img.shields.io/badge/opencv-%235C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/tensorflow_lite-%23FF6F00?style=for-the-badge&logo=tensorflow&logoColor=white"> <img src="https://img.shields.io/badge/gtts-%234285F4?style=for-the-badge&logo=googlecloud&logoColor=white">  <img src="https://img.shields.io/badge/Qt-41CD52?style=for-the-badge&logo=qt&logoColor=white">|<img src="https://img.shields.io/badge/freeRTOS-%6CAE3E?style=for-the-badge&logo=data:image/svg%2bxml;base64,PHN2ZyB3aWR0aD0iMTAwcHgiIGhlaWdodD0iMTAwcHgiIHZpZXdCb3g9IjEwIDMwIDU4IDIwIiB2ZXJzaW9uPSIxLjEiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyIgeG1sbnM6eGxpbms9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkveGxpbmsiPgogICAgPHRpdGxlPkljb24tQXJjaGl0ZWN0dXJlLzY0L0FyY2hfQW1hem9uLUZyZWVSVE9TXzY0PC90aXRsZT4KICAgIDxnIGlkPSJJY29uLUFyY2hpdGVjdHVyZS82NC9BcmNoX0FtYXpvbi1GcmVlUlRPU182NCIgc3Ryb2tlPSJub25lIiBzdHJva2Utd2lkdGg9IjEiIGZpbGw9Im5vbmUiIGZpbGwtcnVsZT0iZXZlbm9kZCI+CiAgICAgICAgPHBhdGggZD0iTTI3Ljk0MywzNy40OTggQzI5LjMyMywzNy40OTggMzAuNDQ1LDM4LjYyMSAzMC40NDUsNDAgQzMwLjQ0NSw0MS4zOCAyOS4zMjMsNDIuNTAzIDI3Ljk0Myw0Mi41MDMgQzI2LjU2Myw0Mi41MDMgMjUuNDQsNDEuMzggMjUuNDQsNDAgQzI1LjQ0LDM4LjYyMSAyNi41NjMsMzcuNDk4IDI3Ljk0MywzNy40OTggTDI3Ljk0MywzNy40OTggWiBNMzUuODQ3LDI5LjU1OCBDMzYuNDAyLDMwLjg4MSAzNy41NzQsMzEuODc4IDM5LDMyLjIwNSBMMzksNTAuODY4IEMzOC4zMjIsNTAuODA3IDM3LjY1MSw1MC42OCAzNyw1MC40OTUgTDM3LDQwIEMzNywzOS40NDggMzYuNTUyLDM5IDM2LDM5IEwzMi4zMjQsMzkgQzMxLjk5OCwzNy41NzEgMzAuOTk2LDM2LjM5NyAyOS42NjcsMzUuODQ0IEMzMC43NzMsMzIuOTc1IDMzLDMwLjcwOSAzNS44NDcsMjkuNTU4IEwzNS44NDcsMjkuNTU4IFogTTQwLDI1LjMyIEM0MS4zOCwyNS4zMiA0Mi41MDIsMjYuNDQzIDQyLjUwMiwyNy44MjMgQzQyLjUwMiwyOS4yMDMgNDEuMzgsMzAuMzI1IDQwLDMwLjMyNSBDMzguNjIsMzAuMzI1IDM3LjQ5OCwyOS4yMDMgMzcuNDk4LDI3LjgyMyBDMzcuNDk4LDI2LjQ0MyAzOC42MiwyNS4zMiA0MCwyNS4zMiBMNDAsMjUuMzIgWiBNNTAuMzM0LDM1Ljg0NCBDNDkuMDA1LDM2LjM5NyA0OC4wMDIsMzcuNTcxIDQ3LjY3NSwzOSBMNDQsMzkgQzQzLjQ0NywzOSA0MywzOS40NDggNDMsNDAgTDQzLDUwLjQ5NiBDNDIuMzUsNTAuNjgxIDQxLjY3OSw1MC44MDcgNDEsNTAuODY4IEw0MSwzMi4yMDUgQzQyLjQyNSwzMS44NzggNDMuNTk4LDMwLjg4IDQ0LjE1MywyOS41NTYgQzQ3LjAwMSwzMC43MDYgNDkuMjI2LDMyLjk3MSA1MC4zMzQsMzUuODQ0IEw1MC4zMzQsMzUuODQ0IFogTTU0LjU1OSw0MCBDNTQuNTU5LDQxLjM4IDUzLjQzNiw0Mi41MDMgNTIuMDU2LDQyLjUwMyBDNTAuNjc2LDQyLjUwMyA0OS41NTQsNDEuMzggNDkuNTU0LDQwIEM0OS41NTQsMzguNjIxIDUwLjY3NiwzNy40OTggNTIuMDU2LDM3LjQ5OCBDNTMuNDM2LDM3LjQ5OCA1NC41NTksMzguNjIxIDU0LjU1OSw0MCBMNTQuNTU5LDQwIFogTTQ1LDQ5LjcgTDQ1LDQxIEw0Ny42NzUsNDEgQzQ3Ljk5Myw0Mi4zOSA0OC45NTEsNDMuNTMzIDUwLjIyNCw0NC4xMDQgQzQ5LjIxMyw0Ni41MDcgNDcuMzQzLDQ4LjUxIDQ1LDQ5LjcgTDQ1LDQ5LjcgWiBNMzEuNDk2LDQ2LjkzMiBDMzEuNDQ4LDQ2Ljg3NSAzMS4zOTksNDYuODIgMzEuMzI2LDQ2LjcyMSBDMzAuNjgxLDQ1LjkxOCAzMC4xNTksNDUuMDQzIDI5Ljc2NSw0NC4xMDkgQzMxLjA0NSw0My41NCAzMi4wMDUsNDIuMzkzIDMyLjMyNCw0MSBMMzUsNDEgTDM1LDQ5LjY5NyBDMzMuNjYyLDQ5LjAyMyAzMi40NjEsNDguMDkyIDMxLjQ5Niw0Ni45MzIgTDMxLjQ5Niw0Ni45MzIgWiBNMjcuNzc1LDQ0LjQ4NiBDMjguMjU0LDQ1LjczOCAyOC45MSw0Ni45MDcgMjkuNzM1LDQ3LjkzMiBDMjkuODA3LDQ4LjAzMyAyOS44OTEsNDguMTMzIDI5Ljk2Nyw0OC4yMjIgQzMxLjMxNCw0OS44NDIgMzMuMDU5LDUxLjExIDM1LDUxLjkwMyBMMzUsNTcgTDM3LDU3IEwzNyw1Mi41NTUgQzM3LjY1Miw1Mi43MTEgMzguMzE4LDUyLjgxOSAzOSw1Mi44NyBMMzksNTcgTDQxLDU3IEw0MSw1Mi44NyBDNDEuNjgyLDUyLjgxOSA0Mi4zNDgsNTIuNzExIDQzLDUyLjU1NiBMNDMsNTcgTDQ1LDU3IEw0NSw1MS45MDMgQzQ4LjMwNyw1MC41MzcgNTAuOTQzLDQ3LjgwNiA1Mi4yMTUsNDQuNDg3IEM1NC42MjQsNDQuNDAxIDU2LjU1OSw0Mi40MjkgNTYuNTU5LDQwIEM1Ni41NTksMzcuNjE3IDU0LjY5MiwzNS42OCA1Mi4zNDUsMzUuNTI3IEM1MS4wNDcsMzEuODAyIDQ4LjE3MiwyOC44OTMgNDQuNDczLDI3LjU0NiBDNDQuMzI4LDI1LjE5NSA0Mi4zODcsMjMuMzIgNDAsMjMuMzIgQzM3LjYxMiwyMy4zMiAzNS42NzIsMjUuMTk1IDM1LjUyNSwyNy41NDcgQzMxLjgyOSwyOC44OTUgMjguOTUzLDMxLjgwNyAyNy42NTYsMzUuNTI3IEMyNS4zMDgsMzUuNjc4IDIzLjQ0LDM3LjYxNiAyMy40NCw0MCBDMjMuNDQsNDIuNDI1IDI1LjM3MSw0NC4zOTUgMjcuNzc1LDQ0LjQ4NiBMMjcuNzc1LDQ0LjQ4NiBaIE0xOCw2MSBMNjIsNjEgTDYyLDIwIEwxOCwyMCBMMTgsNjEgWiBNNjgsMjMgTDY4LDIxIEw2NCwyMSBMNjQsMTkgQzY0LDE4LjQ0OCA2My41NTIsMTggNjMsMTggTDE3LDE4IEMxNi40NDcsMTggMTYsMTguNDQ4IDE2LDE5IEwxNiwyMSBMMTIsMjEgTDEyLDIzIEwxNiwyMyBMMTYsMjcgTDEyLDI3IEwxMiwyOSBMMTYsMjkgTDE2LDMzIEwxMiwzMyBMMTIsMzUgTDE2LDM1IEwxNiwzOSBMMTIsMzkgTDEyLDQxIEwxNiw0MSBMMTYsNDUgTDEyLDQ1IEwxMiw0NyBMMTYsNDcgTDE2LDUxIEwxMiw1MSBMMTIsNTMgTDE2LDUzIEwxNiw1NyBMMTIsNTcgTDEyLDU5IEwxNiw1OSBMMTYsNjIgQzE2LDYyLjU1MyAxNi40NDcsNjMgMTcsNjMgTDYzLDYzIEM2My41NTIsNjMgNjQsNjIuNTUzIDY0LDYyIEw2NCw1OSBMNjgsNTkgTDY4LDU3IEw2NCw1NyBMNjQsNTMgTDY4LDUzIEw2OCw1MSBMNjQsNTEgTDY0LDQ3IEw2OCw0NyBMNjgsNDUgTDY0LDQ1IEw2NCw0MSBMNjgsNDEgTDY4LDM5IEw2NCwzOSBMNjQsMzUgTDY4LDM1IEw2OCwzMyBMNjQsMzMgTDY0LDI5IEw2OCwyOSBMNjgsMjcgTDY0LDI3IEw2NCwyMyBMNjgsMjMgWiIgaWQ9IkFtYXpvbi1GcmVlUlRPU19JY29uXzY0X1NxdWlkIiBmaWxsPSIjRkZGRkZGIj48L3BhdGg+CiAgICA8L2c+Cjwvc3ZnPg==">|

## 주요 문서
|No.|Doc.|Desc.|Notes|
|:---:|:---|:---|:---|
|1|[프로젝트 보고서](./Docs/Project%20Report.md)|프로젝트 전반적인 내용 및 결과 기록| <img src="https://img.shields.io/badge/현재 작성 중-%23FF6F00?style=for-the-badge&logoColor=white">|
|2|[안내 로봇 문서](./Robot/README.md)|안내 로봇 사용법 및 개발 내용 정리| <img src="https://img.shields.io/badge/작성 예정-%23E7352C?style=for-the-badge&logoColor=white">|
|3|[안내 손잡이 문서](./Handle/README.md)|안내 손잡이 사용법 및 개발 내용 정리| <img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">|
|4|[Setup 프로그램 문서](./Qt/README.md)|대여자 초기 세팅 메뉴얼| <img src="https://img.shields.io/badge/작성 완료-%41CD52?style=for-the-badge&logoColor=white">|

## 문서 이력
|Ver.|Desc.|Author|Date|Notes|
|:---:|:---|:---:|:---:|:---|
|0.1|README.md 생성|정현우|2024-04-09|git 생성|
|0.2|주제 선정, 배경 및 기획 추가|정현우|2024-04-22|중간 발표|
|0.3|README.md 구조 변경|정현우|2024-05-22|프로젝트 종료|
|0.4|기술 스택 및 문서 추가|정현우|2024-05-27||