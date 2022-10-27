#!/usr/bin/python3           # This is client.py file

import socket
import re


def client_program():

    sentence = "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,1"
    len1 = len(sentence)

    ans = ord(sentence[0])
 
    for i in range(1,len1):
 
        # Traverse string to find the XOR
        ans = (ans ^ (ord(sentence[i])))
 
    # Return the XOR
    
    print(str(hex(ans))[-2:])


if __name__ == '__main__':
    client_program()