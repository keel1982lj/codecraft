class Cross(object):
    def __init__(self, id, rid1=None, rid2=None, rid3=None, rid4=None, mapid=-1):
        self.id = id
        self.rids = [rid1, rid2, rid3, rid4]
        self.mapid = mapid
        self.time = 0