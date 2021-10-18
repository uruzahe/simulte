import sys
import os

if __name__ == "__main__":
    sca_file_name = sys.argv[1]
    vec_file_name = sys.argv[2]
    ini_file_path = "./../../vfc_in_local.ini" if len(sys.argv) <= 3 else sys.argv[3]

    if sca_file_name.split(".")[0] != vec_file_name.split(".")[0]:
        raise Exception(f"Invalid sca and vec, \n sca: {sca_file_name} \n vec: {vec_file_name}.")

    commo_name = sca_file_name.split(".")[0]
    csv_file_name = commo_name + ".csv"
    ini_file_name = commo_name + ".ini"

    print(f"sca_file: {sca_file_name}")
    print(f"vec_file: {vec_file_name}")
    print(f"csv_file: {csv_file_name}")
    print(f"ini_file: {ini_file_name}")
    print(f"original_ini_file_path: {ini_file_path}")

    os.system(f"scavetool x {sca_file_name} {vec_file_name} -o {commo_name}.csv -v")
    os.system(f"cp -rf {ini_file_path} {commo_name}.ini")
