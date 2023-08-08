from csv import writer
import time
from csv import DictWriter
from datetime import datetime
field_names = ['ID', 'NAME', 'RANK',
               'ARTICLE', 'COUNTRY']

dict = {'ID': 6, 'NAME': 'William', 'RANK': 5532,
        'ARTICLE': 1, 'COUNTRY': 'UAE'}
filename = datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_' +'event.csv'
file = open(filename, 'a')
# Pass this file object to csv.writer()
# and get a writer object
dictwriter_object = DictWriter(file, fieldnames=field_names)

# Pass the list as an argument into
# the writerow()
dictwriter_object.writeheader()
dictwriter_object.writerow(dict)

# Close the file object
file.close()