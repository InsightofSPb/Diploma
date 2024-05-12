import os
import shutil

# Исходная папка с файлами
source_folder = "/home/s/dipl/src/hector_quadrotor/controller/results/19_56_12/base"

# Папка назначения для сохранения каждого 10-го файла
destination_folder = "results/19_56_12/small"

# Получаем список всех файлов в исходной папке
files = os.listdir(source_folder)

# Сортируем файлы по имени
files.sort()

# Перебираем файлы с шагом 10
for i in range(0, len(files), 20):
    file_name = files[i]
    source_path = os.path.join(source_folder, file_name)
    destination_path = os.path.join(destination_folder, file_name)
    
    # Копируем файл в папку назначения
    shutil.copy2(source_path, destination_path)
    print(f"Файл {file_name} скопирован в {destination_folder}")