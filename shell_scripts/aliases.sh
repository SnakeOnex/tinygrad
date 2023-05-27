alias cd_latest_log='[ -z "$BROS_PATH" ] && echo "BROS_PATH is not set" || cd "$(ls -td $BROS_PATH/data/bros_logs/*/ | head -n 1)"'
