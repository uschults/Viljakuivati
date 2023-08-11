# For saving data to clientpi, serverpi and to cloud in the future.
import csv


def save_to_client (file_name, data_value):
    with open('/home/pi/' + file_name, 'w') as open_file:
        writer = csv.writer(open_file)
        writer.writerow()
     

def save_to_server ():
    pass

def save_to_cloud ():
    pass

def main():
    pass

if __name__ == "__main__":
    main()