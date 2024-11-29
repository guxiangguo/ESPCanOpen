import json
import os

def merge_files(json_file):
    # 读取 flasher_args.json 文件
    with open(json_file, "r") as f:
        data = json.load(f)

    # 获取 flash_files 字段
    flash_files = data["flash_files"]
    # 获取脚本文件所在文件夹的路径和文件夹名称
    script_folder_path = os.path.dirname(os.path.abspath(__file__))
    script_folder_name = os.path.basename(script_folder_path)

    # 创建一个新文件来存储合并后的内容，输出文件名设置为上一级文件夹名称加上 ".bin" 后缀
    output_file = os.path.join(script_folder_path, script_folder_name + ".bin")
    print("output_file name:", script_folder_path)
    with open(output_file, "wb") as merged_file:
        for offset, file_path in flash_files.items():
            # 跳过可能包含父目录的路径
            file_path = os.path.join(os.path.dirname(json_file), file_path)
            with open(file_path, "rb") as f:
                file_data = f.read()
                # 将文件数据写入到相应的偏移位置
                print("offset:",offset)
                print("file_path:", file_path)
                merged_file.seek(int(offset, 16))

                merged_file.write(file_data)

    print("文件合并完成，输出到:", output_file)

if __name__ == "__main__":
    json_file = "build/flasher_args.json"
    merge_files(json_file)