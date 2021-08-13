
def runTimeToCounter(time):
    minutes = int(time // 60)
    seconds = str(int(time - (minutes*60)))
    if len(seconds) == 1:
        seconds = "0" + seconds
    return "{0}-{1}".format(minutes, seconds)
