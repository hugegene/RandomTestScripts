import os

base = "/media/eugene/"
disk = ""
for folder in os.listdir(base):
    for subdir in os.listdir(os.path.join(base, folder)): 
        #print(subdir)
        if subdir == "do_not_delete":
            print("found")
            disk = os.path.join(base, folder)
            
print(disk)
