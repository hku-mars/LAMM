import os

def rename_files(directory):
    file_list = os.listdir(directory)
    file_list.sort()  # 按文件名排序

    for i, filename in enumerate(file_list, start=1):
        if os.path.isfile(os.path.join(directory, filename)):
            extension = os.path.splitext(filename)[1]  # 获取文件扩展名
            new_filename = str(i) + extension  # 将文件名改为数字
            os.rename(os.path.join(directory, filename), os.path.join(directory, new_filename))
            print(f"Renamed {filename} to {new_filename}")

# 修改文件夹路径为你想要修改文件名的文件夹
directory_path = "/home/weihairuo/bag/huawei/1"

rename_files(directory_path)
