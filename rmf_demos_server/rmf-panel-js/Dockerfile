# 1. Node.js 기반 빌드 단계
FROM node:18 AS builder

# 작업 디렉토리 설정
WORKDIR /app

# 프로젝트 코드 복사
COPY rmf_panel /app/rmf_panel

# 의존성 설치 및 빌드 실행
RUN npm install --prefix rmf_panel
RUN npm run build --prefix rmf_panel

# 2. Python 서버 이미지
FROM python:3.9-slim

# 작업 디렉토리 설정
WORKDIR /app

# 빌드된 정적 파일 복사
COPY --from=builder /app/rmf_panel/dist /app/rmf_panel

# 컨테이너가 3000번 포트에서 실행되도록 설정
EXPOSE 3000

# Python HTTP 서버 실행
CMD ["python3", "-m", "http.server", "3001", "--directory", "/app/rmf_panel/rmf_panel"]
