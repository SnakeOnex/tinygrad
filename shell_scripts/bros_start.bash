# NOTE: This script has to be run a from the bros/.. directory
# save name of folder in a variable
folder_name=$(date +"%d-%m-%Y__%H:%M")

# if name is zotac else
if [ "$HOSTNAME" = "zotac" ]; then
    folder_path="/home/zotac/bros/data/bros_logs/$folder_name"
    output_log_path="/home/zotac/bros/data/bros_logs/$folder_name/output.log"
    cd bros/
else
    folder_path="data/bros_logs/$folder_name"
    output_log_path="data/bros_logs/$folder_name/output.log"
fi

mkdir $folder_path
python3 master.py --log_folder $folder_path 2>&1 | tee $output_log_path
