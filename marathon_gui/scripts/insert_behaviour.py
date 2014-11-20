#!/usr/bin/env python

import sys
import std_msgs.msg
from mongodb_store.message_store import MessageStoreProxy
import yaml
import json
import pprint


def loadDialogue(inputfile, dataset_name):
    print "openning %s" % inputfile
    with open(inputfile) as f:
            content = f.readlines()
    print "Done"
    return content


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "usage: insert_behaviour input_file.txt dataset_name"
        sys.exit(2)

    filename = str(sys.argv[1])
    dataset_name = str(sys.argv[2])
    msg_store = MessageStoreProxy(collection="hri_behaviours")
    data = yaml.load(open(filename))
    pprint.pprint(data)
    meta = {}
    meta["collection"] = dataset_name
    msg_store.insert(std_msgs.msg.String(json.dumps(data)), meta)
