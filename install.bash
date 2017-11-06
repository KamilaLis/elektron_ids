#!/bin/bash
ssh -oHostKeyAlgorithms='ssh-rsa' stero@192.168.1.122
ssh-keygen -t rsa
ssh-copy-id stero@192.168.1.122