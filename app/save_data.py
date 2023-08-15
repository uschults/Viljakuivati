# For saving data to clientpi, serverpi and to cloud in the future.
import csv


def save_to_client (file_name, data_value):
    with open(file_name + ".csv", 'a+') as open_file:
        writer = csv.writer(open_file)
        writer.writerow(list(data_value))
     

def save_to_server ():
    pass

def save_to_cloud ():
    pass

def main():
    pass

if __name__ == "__main__":
    main()