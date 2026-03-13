import bagpy
from bagpy import bagreader

b = bagreader('2024-06-17-16-44-21.bag')
csv_files = b.convert_csv()
