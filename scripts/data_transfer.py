import shutil
import os

source_folder = '~/dev/ATTACKS-SW/Datasets'
destination_folder = '/media/attacks/6C09-B902'


os.makedirs(destination_folder, exist_ok=True)
files = os.listdir(source_folder)

for file_name in files:
    source_file = os.path.join(source_folder, file_name)
    destination_file = os.path.join(destination_folder, file_name)
    if os.path.isfile(source_file):
        print(f"Moving {file_name} to {destination_folder}")
        shutil.move(source_file, destination_file)
print("Done.")
