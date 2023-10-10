import pandas as pd

if __name__ == "__main__":
    base_path = '/home/pete/PycharmProjects/AdaptiveDrone/logs/09_10_field/'
    save_path = '/home/pete/PycharmProjects/AdaptiveDrone/images/test_plots/'
    path = base_path + 'test_working.csv'
    pd.read_csv(path)