from pprint import pprint
from copy import deepcopy
class DataHolder:
    def __init__(self, data: list):
        self.__dict__['data'] = deepcopy(data)
        for key in self.data.keys():
            setattr(self, key, self.data[key])
    def display_data(self, attr_names=None):
        if attr_names is None:
            pprint(self.data)
        else:
            pprint(self.data[attr_names])

    def get_data(self):
        return self.data
    def __setattr__(self, key, value):
        self.__dict__[key] = value
        self.data[key] = value

if __name__ == "__main__":
    data_holder = DataHolder({'x': 'y'})
    data_holder.display_data()