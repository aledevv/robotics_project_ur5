# TODO dato che le immagini sono fornite in cartelle e hanno tutte lo stesso nome... questo script
#      si occupa di:
# 1.   rinominare le immagini
# 2.   creare uno zip contenente le cartelle train, val, test


# WARNING cambia nome cartella sull'altro quandro ricrei il JSON

import os
import shutil
import numpy as np

def main():
    
    #create new folder
    #os.mkdir('dataset')
    
    
    img_idx = 0         #image index
    main_folder = 'assigns'
    for assign_folder in os.listdir(main_folder):
        if not assign_folder.startswith('.'):
            for scene in os.listdir(main_folder+'/'+assign_folder):
                if not scene.startswith('.'):
                    for file in os.listdir(main_folder+'/'+assign_folder+'/'+scene):
                        if file.endswith('.json'): # images we want gave the same name of json files
                            image_name = file[:len(file)-3]+'peg'
                            shutil.copy(main_folder+'/'+assign_folder+'/'+scene+'/'+image_name, 'dataset/')
                            os.renames('dataset/'+image_name,'dataset/img_'+str(img_idx)+'.jpeg')
                            img_idx+=1
                            

if __name__ == "__main__":
    main()