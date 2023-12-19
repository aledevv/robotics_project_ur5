import json
import os
import numpy as np
import cv2

###################################
#          GLOBAL VARS            #
###################################

images_list_dict = []           # dictionary containing all the images of the dataset
annotations_list_dict = []      # dictionary containing all the annotations of the object inside the images

###################################
#           FUNCTIONS             #
###################################

def extractAnnotations(json_file_path, img_id, annotation_id):
    with open(json_file_path) as json_data:     # open the passed json file
        data = json.load(json_data)
    
    num_of_objects=len(data)            # num of objects in the image
    
    #for each object, get useful data and write its annotation
    for current_obj in data:
        # get label
        label = data[current_obj]['y']
        #get bbox
        bbox = data[current_obj]['bbox']
        #write the annotation
        newAnnotation(annotation_id, img_id, label, bbox)
        annotation_id+=1
        
    return annotation_id
    
    
def writeInJSONFile(jsonToWrite):
    with open('_annotations.coco.json') as ann:
        ann.write(jsonToWrite)
    



''' IMAGES has fields:
- id: (progressive)
- licence: 1
- file_name: (the name of the img)
- height: (img height)
- width: (img width)
- date_captured: x
'''

def newImage(image_name, img_id, height, width):
    
    img_dict = {'id': img_id,
                'licence': 1,
                'file_name': image_name,
                'height': height,
                'width': width,
                'date_captured': 'x'}
    
    addToImagesDict(img_dict)


def addToImagesDict(dictToAdd): # Use dictionaries that are as json
    images_list_dict.append(dictToAdd)




''' ANNOTATIONS has fields:
- id: (progressive)
- image_id: (id of one of the images in IMAGES)
- category_id: (label in classes vector)
- bbox: (vector of 4 values [x0,y0,width,height])
- area: (bbox area)
- segmentation: []
- iscrowd: 0
'''    



def newAnnotation(id, image_id, category_id, bbox):
    ann_dict = {'id': id,
                'image_id': image_id,
                'category_id': category_id,
                'bbox': bbox,
                'area': bbox[2]*bbox[3], # Area = width * height -> they are respectively at 3rd and 4th index of the bbox
                'segmentation': [],
                'iscrowd': 0}
    addToAnnotations(ann_dict)


def addToAnnotations(annToAdd): # Use dictionaries that are as json
    annotations_list_dict.append(annToAdd)
   
   
   
   
   
   
   
''' #TO GET THE NAME OF THE CLASSES
def writeClasses(data):
    for current_obj in data:
        classes.append(data[current_obj]['y'])
        #print(data[current_obj]['y'])'''



def main():

    classes = [ 'X1-Y1-Z2' 'X1-Y2-Z1' 'X1-Y2-Z2' 'X1-Y2-Z2-CHAMFER' 'X1-Y2-Z2-TWINFILLET'
                'X1-Y3-Z2' 'X1-Y3-Z2-FILLET' 'X1-Y4-Z1' 'X1-Y4-Z2' 'X2-Y2-Z2'
                'X2-Y2-Z2-FILLET']
    # TODO RICORDATI DI VERIFICARE CHE L'INDICE MESSO SIA GIUSTO (MAGARI QUI PARTE DA 0)
    
    
    annotations_json = {'images': [],
                       'annotations':[]}
    
    
    
    # lets get all data
    main_folder = 'assigns'

    #print(os.path.join(main_folder, 'assign1'))

    img_id=0
    annotation_id=0    #annotation id
    for assign_folder in os.listdir(main_folder):
        if not assign_folder.startswith('.'):
            for scene in os.listdir(main_folder+'/'+assign_folder):
                if not scene.startswith('.'):
                    for file in os.listdir(main_folder+'/'+assign_folder+'/'+scene):
                        if file.endswith('.json'):
                            
                            json_file = main_folder+'/'+assign_folder+'/'+scene+'/'+file
                            
                            #image and json file have the same name, just setting the extension .jpeg to open the img afterwards
                            img_file = file[:len(file)-4]+'jpeg'
                            
                            #open the relative image
                            opened_img = cv2.imread(main_folder+'/'+assign_folder+'/'+scene+'/'+img_file,0)
                            height, width = opened_img.shape[:2]
                            
                            #add image to images section of the json
                            #newImage(img_file, img_id, height, width)
                            
                            annotation_id=extractAnnotations(json_file, img_id, annotation_id)
                            
                            img_id+=1
    
    #print(annotations_list_dict)                            
    #file = open('out.txt', 'w')
    #file.write(str(annotations_list_dict))
    
    annotations_json['images']=images_list_dict
    annotations_json['annotations']=annotations_list_dict
    
    writeInJSONFile(annotations_json)                       
    
if __name__ == "__main__":
    main()