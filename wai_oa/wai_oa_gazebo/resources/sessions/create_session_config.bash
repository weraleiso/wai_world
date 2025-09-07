#!/bin/bash

script_file_path_real=$(dirname "$(realpath "$0")")

pptx_file_path_full="$1"
pptx_file_path_real=$(dirname "$(realpath "$1")")
pptx_file_name="${pptx_file_path_full##*/}"
pptx_file_name_base="${pptx_file_name%.*}"
pptx_file_notes_path="$pptx_file_path_real/ppt/notesSlides"

echo "===================================="
echo "=== CREATING SESSION CONFIG FILE ==="
echo "===================================="
echo "PRESENTATION file: $pptx_file_path_full"
echo "OUTPUT to: $pptx_file_name_base/$pptx_file_name_base.yaml"

unzip -qq -o $pptx_file_path_full -d $pptx_file_path_real

cd $pptx_file_notes_path
readarray -d '' entries < <(printf '%s\0' *.xml | sort -zV)

cd $script_file_path_real
cat session_config_template.yaml > $pptx_file_name_base/$pptx_file_name_base.yaml

for pptx_file_notes_iterator in "${entries[@]}"; do

    # OPTIONAL - Remove specific file content on demand before XML node query:
    # awk '{gsub("<a:rPr lang=\"en-US\" dirty=\"0\"/>",""); print}' $pptx_file_notes_path/$pptx_file_notes_iterator > temp.fil && mv temp.fil $pptx_file_notes_path/$pptx_file_notes_iterator

    pptx_file_notes_number=${pptx_file_notes_iterator//[^0-9]/}
    # xml_query=$(xmlstarlet sel -t -m "//p:txBody/a:p/a:r/a:t" -v . -n "$pptx_file_notes_path/$pptx_file_notes_iterator")
    xml_query=$(xmlstarlet sel -t -m "//p:txBody//a:p[.//a:r//a:t]" -v . -n "$pptx_file_notes_path/$pptx_file_notes_iterator")
    if [ -z "$xml_query" ]; then
        echo "Found EMTPY scene config, replacing with default!"
        xml_query=$(echo -e "teleprompter: Add slide notes here\ncamera: default")
    fi
    xml_query_indented="$(echo "$xml_query" | sed -e 's/^/    /')"
    echo -e "setup_scene_$pptx_file_notes_number:" >> $pptx_file_name_base/$pptx_file_name_base.yaml
    echo -e "$xml_query_indented\n" >> $pptx_file_name_base/$pptx_file_name_base.yaml
done

echo "------------------------------------"
echo "--- SESSION CONFIG FILE CREATED! ---"
echo "------------------------------------"
