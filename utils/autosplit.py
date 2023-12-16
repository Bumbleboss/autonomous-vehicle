import os
import math
import argparse

def get_weighted_list(images_dir, weights):
  img_arr = os.listdir(images_dir)
  img_len = len(img_arr)

  train_i = math.floor(img_len * weights[0])
  valid_i = math.floor(img_len * weights[1])
  test_i = math.floor(img_len * weights[2])

  print('Used %d out of %d images' % (train_i + valid_i + test_i, img_len))

  return [
    img_arr[:train_i],
    img_arr[train_i:train_i + valid_i],
    img_arr[train_i + valid_i:train_i + valid_i + test_i]
  ]

def create_dir(dataset_dir, category_dir, subset_dir):
  for category in category_dir:
    for subset in subset_dir:
      computed_path = os.path.join(dataset_dir, category, subset)

      if not os.path.isdir(computed_path):
        os.makedirs(computed_path)
        print('Directory does not exist, creating directory: %s' % computed_path)

def main():
  weights = (0.9, 0.1, 0.0)
  parser = argparse.ArgumentParser(description='Autosplit images')

  parser.add_argument('-d', '--dataset', help='dataset directory name', type=str, required=True)
  parser.add_argument('-i', '--image', help='relative images directory path', type=str, required=True)
  parser.add_argument('-l', '--label', help='relative labels directory path', type=str, required=True)
  parser.add_argument('-y', '--yolo', help='yolo version', type=int, required=True)
  parser.add_argument('-w', '--weights', help='weight distribution', nargs='+', type=float, default=weights)

  args = parser.parse_args()

  dataset = args.dataset
  images_d = args.image
  labels_d = args.label
  weights = tuple(args.weights)

  if (args.yolo == 7):
    category_d = ['train', 'valid', 'test']
    subset_d = ['images', 'labels']
  else:
    category_d = ['images', 'labels']
    subset_d = ['train', 'valid', 'test']
    
  # Create directories if they don't exist
  create_dir(dataset, category_d, subset_d)

  # Create weighted list of images by providing its directory
  weighted_list = get_weighted_list(os.path.join(dataset, images_d), weights)

  for i, category in enumerate(weighted_list):
    for file in category:
      org_img = os.path.join(dataset, images_d, file)
      org_txt = os.path.join(dataset, labels_d, file)
      org_txt = os.path.splitext(org_txt)[0]+'.txt'

      if (args.yolo == 7):
        image_path = os.path.join(dataset, category_d[i], subset_d[0], file)
        txt_path = os.path.join(dataset, category_d[i], subset_d[1], file)
      else:
        image_path = os.path.join(dataset, category_d[1], subset_d[i], file)
        txt_path = os.path.join(dataset, category_d[0], subset_d[i], file)
      
      txt_path = os.path.splitext(txt_path)[0]+'.txt'

      os.popen('cp %s %s' % (org_img, image_path))
      os.popen('cp %s %s' % (org_txt, txt_path))

if __name__ == "__main__":
  main()