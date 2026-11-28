#!/bin/bash
# 반디 외부 접속 터널 실행 — 맥에서 브로커(5002)와 함께 켜 둔다. 끄면 외부 주소는 "브로커 꺼짐" 화면.
# 터널 bandi(44c447ef…)는 Workers VPC 서비스 bandi-broker(→ 127.0.0.1:5002)가 쓴다.
exec npx -y wrangler@4 tunnel run 44c447ef-5854-4eab-9672-04a599e995e5
