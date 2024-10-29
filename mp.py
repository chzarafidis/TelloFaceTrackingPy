import multiprocessing


def worker():
    print('Worker')


if __name__ == '__main__':
    p = multiprocessing.Process(target=worker)
    p.start()