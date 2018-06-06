import multiprocessing
import cv2



class TopCamera:
    def __init__(self):
        pass

    def camera_recording(self, event):
        cam_paper = cv2.VideoCapture(0)
        #global cam_paper, video_write
        ret_paper, frame_paper = cam_paper.read()
        h, w = frame_paper.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        video_write = cv2.VideoWriter('saved_out.avi', fourcc, 30.0, (w, h))
        while (cam_paper.isOpened()):
            if event.is_set():
                cam_paper.release()
                video_write.release()
                cv2.destroyAllWindows()
                event.clear()
            ret_paper, frame_paper = cam_paper.read()
            frame_paper = cv2.flip(frame_paper, 1)

            if ret_paper == True:
                video_write.write(frame_paper)
                cv2.imshow('paper', frame_paper)
            else:
                break
            cv2.waitKey(1)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break

    def start_recording_proc(self):
        global proc
        proc = multiprocessing.Process(target = camera_recording, args=(event, ))
        proc.start()

    def stop_recording(self, cam_paper, video_write, event):
        event.set()
        proc.start()
        cam_paper.release()
        video_write.release()
        cv2.destroyAllWindows()


