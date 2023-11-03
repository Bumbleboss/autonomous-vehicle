from ultralytics.data.utils import autosplit

autosplit('datasets/udacity-self-driving/images', (0.7, 0.15, 0.15))
