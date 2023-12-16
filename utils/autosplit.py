import os
import math


def get_weighted_list(images_dir, weights=(0.9, 0.1, 0.0)):
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
            computed_path = os.path.join(dataset_dir, subset, category)
            if not os.path.isdir(computed_path):
                os.makedirs(computed_path)
                print('Directory does not exist, creating directory: %s' % computed_path)


dataset = 'Self-Driving-Car-3'
images_d = 'export/images'
labels_d = 'export/labels'
category_d = ['images', 'labels']
subset_d = ['train', 'valid', 'test']


def main():
    # Create directories if they don't exist
    create_dir(dataset, category_d, subset_d)

    # Create weighted list of images by providing its directory
    weighted_list = get_weighted_list(os.path.join(dataset, images_d), (0.6, 0.2, 0.2))

    for i, category in enumerate(weighted_list):
        for file in category:
            org_img = os.path.join(dataset, images_d, file)
            org_txt = os.path.join(dataset, labels_d, file)
            org_txt = os.path.splitext(org_txt)[0]+'.txt'

            image_path = os.path.join(dataset, category_d[0], subset_d[i], file)
            txt_path = os.path.join(dataset, category_d[1], subset_d[i], file)
            txt_path = os.path.splitext(txt_path)[0]+'.txt'

            os.popen('cp %s %s' % (org_img, image_path))
            os.popen('cp %s %s' % (org_txt, txt_path))


if __name__ == "__main__":
    main()
