


def setMsgAttr(msg, attr, value):
    ''' attrs, value list or single '''
    if isinstance(attr, list) and isinstance(value, list):
        for a,v in zip(attr, value):
            setattr(msg, a, v)
    else:
        setattr(msg, attr, value)


def getMsgAttr(msg, attr):
    if isinstance(attr, list):
        return [getattr(msg, a) for a in attr]
    return getattr(msg, attr)

