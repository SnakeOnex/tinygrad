alias cd_latest_log='[ -z "$BROS_PATH" ] && echo "BROS_PATH is not set" || cd "$(ls -td $BROS_PATH/data/bros_logs/*/ | head -n 1)"'
alias gen_latest_log='[ -z "$BROS_PATH" ] && echo "BROS_PATH is not set" || python3 $BROS_PATH/scripts/generate_all_charts.py --latest --video --bros_data $BROS_PATH/data/bros_logs'
