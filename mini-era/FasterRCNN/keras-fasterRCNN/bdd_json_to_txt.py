import json

## The first version of the benchmark classifies 20 categories

labels = [
   'building', 
   'fence', 
   'lane divider',
   'parking sign',
   'rail track',
   'sidewalk',
   'pole'     
   'street light',
   'traffic cone',
   'traffic light',
   'traffic sign',
   'vegetation',
   'person',   
   'bicycle',  
   'bus',      
   'car',      
   'motorcycle',
   'trailer',  
   'train',    
   'truck'    
]

# Let's train it on 10000 images instead of 100k.

num_training_images = 10000
training_image_filepath = '~/Datasets/bdd100k_reduced/images/train/'
training_labels_file = '/home/nandhinic/Datasets/bdd100k_reduced/labels/bdd100k_reduced_labels_images_train.json'
def main():
    with open(training_labels_file) as json_file:
        print('Loading json file ...')
        data = json.load(json_file)
        for n in range(0,num_training_images):
            name = data[n]['name']
            print('Generating annotations for image',n, ' ' , name)
            for l in data[n]['labels']:
                if l['category'] in labels: ## as of now restricting it to 20 class labels
                    desc = training_image_filepath + name + ',' + str(l['box2d']['x1']) + ',' + str(l['box2d']['y1']) + ',' + str(l['box2d']['x2']) + ',' + str(l['box2d']['y2']) + ',' + l['category'] + '\n'
                    with open("bdd100k_annotate.txt","a") as af:
                        af.write(desc)
    

if __name__ == "__main__":
    main()
