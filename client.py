'''
A file to emulate the types of requests and responses the API will receive to test functionality.
'''

import requests

def main():
    '''The main function in which this dummy code will operate.'''
    resp = requests.get('http://127.0.0.1:8000')
    print(resp.text)


if __name__ == '__main__':
    main()
