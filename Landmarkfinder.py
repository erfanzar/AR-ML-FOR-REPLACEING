def LmFinder(camera, result):
    lmlist = []
    if result.multi_hand_landmarks:
        myhand = result.multi_hand_landmarks[0]
        for id, lm in enumerate(myhand.landmark):
            h, w, _ = camera.shape
            x = int(lm.x * w)
            y = int(lm.y * h)
            lmlist.append([id, x, y])
    return lmlist
