def csv_write_f():

    flag = True
    filename = ""

    def write(x,y,z):

        import datetime
        import csv

        nonlocal flag
        nonlocal filename

        if flag:
            now_time = datetime.datetime.now()
            filename = 'test_' + now_time.strftime('%Y%m%d_%H%M%S') + '.csv'

            with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z"])
            flag = False


        with open(filename,'a',newline='') as f: 
                writer = csv.writer(f)
                writer.writerow([x, y, z])

    return write

csv_write = csv_write_f()


data = (1,2,3)
csv_write(*data)


