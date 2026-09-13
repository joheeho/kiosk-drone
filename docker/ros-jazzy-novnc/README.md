# ROS2 Jazzy noVNC Docker
<img width="1919" height="1079" alt="image" src="https://github.com/user-attachments/assets/b4d60d66-627c-4739-bc7a-4949ed8917fe" width="175" height="90" />

브라우저만으로 ROS2 Jazzy(Ubuntu 24.04) 데스크톱 환경에 접속할 수 있는 Docker 이미지입니다.
별도의 ROS2 설치 없이, Docker만 있으면 noVNC를 통해 브라우저로 바로 실습 환경을 띄울 수 있습니다.

## 환경 정보
- Ubuntu 24.04 
- ROS2 배포판: **Jazzy** (Ubuntu 24.04 기반)
- 컨테이너 접속 시 터미널을 열면 ROS2 환경이 자동으로 설정되어 있습니다. (`source /opt/ros/jazzy/setup.bash` 자동 적용)
- `ROS_DOMAIN_ID=30`으로 설정되어 있습니다. (다른 실습생과 같은 네트워크에서 토픽이 섞이지 않도록 하기 위함이니, 임의로 바꾸지 마세요.)


## 사전 준비물

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Windows / Mac) 또는 Docker Engine (Linux)

설치 확인:
```bash
docker --version
docker compose version
```
설치후 반드시 Docker Desktop 실행:
```
win키 + Docker Desktop 검색 후 실행
```
## 실행 방법

### 1. 이 저장소 클론

```bash
git clone https://github.com/joheeho/kiosk-drone.git
cd kiosk-drone/docker/ros-jazzy-novnc
```

### 2. 컨테이너 실행

```bash
docker compose up -d
```

처음 실행 시 이미지를 다운로드하므로 몇 분 정도 걸릴 수 있습니다. 두 번째 실행부터는 즉시 시작됩니다.

### 3. 브라우저로 접속

컨테이너가 뜨면 브라우저에서 아래 주소로 접속합니다.

```
http://localhost:8080/
```

브라우저에서 `http://localhost:8080/` 접속하면 데스크톱 화면이 뜹니다.
noVNC의 연결 버튼을 클릭하여 Ubuntu를 자유롭게 사용합니다.

### 4. 종료

```bash
docker compose down
```

## 실습 파일 저장 위치(중요!!)

`workspace` 폴더가 컨테이너 바탕화면(`~/Desktop`)과 연결되어 있습니다. 
(`~/Desktop`) 밖에 디렉토리에 정보는 저장되지 않습니다!! 실습 코드는 (`~/Desktop`) 안에 저장하세요.
- **컨테이너 안에서 바탕화면에 작업한 내용은** → 로컬 `workspace` 폴더에 그대로 저장됩니다.

즉, 컨테이너를 껐다 켜거나 삭제해도 **`workspace` 폴더 안의 내용은 사라지지 않습니다.**

실습 코드는 반드시 바탕화면(또는 그 하위 폴더)에 저장하세요.

# 🔄 자주 사용하는 명령어

### 실행

```bash
docker compose up -d
```

### 종료

```bash
docker compose down
```

### 실행 상태 확인

```bash
docker compose ps
```

### 실시간 로그 확인

```bash
docker compose logs -f
```

### 컨테이너 재시작

```bash
docker compose restart
```

---

# 🛠️ 문제 해결

## 1. `docker: command not found`

Docker가 설치되어 있지 않거나 PATH가 설정되지 않은 경우입니다.

Docker가 정상적으로 설치되어 있는지 확인하세요.

```bash
docker --version
```

Windows / macOS에서는 Docker Desktop이 실행 중인지 확인하세요.

---

## 2. `docker compose` 명령어가 실행되지 않는 경우

다음 명령어를 실행해 보세요.

```bash
docker compose version
```

버전 정보가 나오지 않는다면 Docker Desktop 또는 Docker Compose Plugin 설치 상태를 확인하세요.

---

## 3. `localhost`에 접속할 수 없는 경우

먼저 컨테이너가 실행 중인지 확인합니다.

```bash
docker compose ps
```

컨테이너가 실행되고 있지 않다면:

```bash
docker compose up -d
```

를 다시 실행합니다.

그래도 문제가 발생한다면 로그를 확인합니다.

```bash
docker compose logs
```

---

## 4. 8080 포트를 사용할 수 없다는 오류가 발생하는 경우

다른 프로그램이 이미 8080번 포트를 사용하고 있을 수 있습니다.

`docker-compose.yaml`의:

```yaml
ports:
  - "8080:80"
```

부분을 다음과 같이 변경할 수 있습니다.

```yaml
ports:
  - "XXXX:80"
```

그 후:

```bash
docker compose up -d
```

를 실행하고 웹 브라우저에서 다음 주소로 접속합니다.

```text
http://localhost:XXXX
```

---

# 🧹 Docker 환경을 완전히 삭제하고 싶은 경우

컨테이너를 종료하고 삭제합니다.

```bash
docker compose down
```

Docker 이미지까지 삭제하려면:

```bash
docker rmi cire21st/ros-jazzy-full:latest
```

> `workspace` 폴더는 Docker 컨테이너와 별도로 로컬 컴퓨터에 존재하므로 이미지나 컨테이너를 삭제해도 파일이 삭제되지 않습니다.

---

## License

This project is provided for educational and development purposes.
