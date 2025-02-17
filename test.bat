@echo off
setlocal

:: 设置LEADERBOARD_ROOT为脚本所在目录
set "LEADERBOARD_ROOT=%~dp0"

:: 激活Conda环境
call conda activate carla

:: 设置其他环境变量
set TEAM_AGENT=%LEADERBOARD_ROOT%leaderboard/autoagents/human_agent.py
set ROUTES=%LEADERBOARD_ROOT%data/routes_devtest.xml
set ROUTES_SUBSET=0
set REPETITIONS=1
set DEBUG_CHALLENGE=1
set CHALLENGE_TRACK_CODENAME=SENSORS
set CHECKPOINT_ENDPOINT=%LEADERBOARD_ROOT%results.json
set RECORD_PATH=
set RESUME=

:: 执行Python脚本
python %LEADERBOARD_ROOT%leaderboard/leaderboard_evaluator.py ^
  --routes=%ROUTES% ^
  --routes-subset=%ROUTES_SUBSET% ^
  --repetitions=%REPETITIONS% ^
  --track=%CHALLENGE_TRACK_CODENAME% ^
  --checkpoint=%CHECKPOINT_ENDPOINT% ^
  --debug-checkpoint=%DEBUG_CHECKPOINT_ENDPOINT% ^
  --agent=%TEAM_AGENT% ^
  --agent-config=%TEAM_CONFIG% ^
  --debug=%DEBUG_CHALLENGE% ^
  --record=%RECORD_PATH% ^
  --resume=%RESUME%

endlocal
