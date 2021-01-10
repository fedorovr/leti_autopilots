from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        obs, _, _, _ = env.step([0, 0])
        img = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)

        while not self.is_duck_in_front(img):
            obs, reward, done, info = env.step([1, 0])
            img = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
            env.render()

    def is_duck_in_front(self, img):
        lower_range = np.array([0, 150, 150])      # BGR
        upper_range = np.array([100, 255, 255])    # BGR
        mask = cv2.inRange(img, lower_range, upper_range)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for line in contours:
            x, y, w, h = cv2.boundingRect(line)
            if w > 100 and h > 100:  # Something big and yellow is considered as a duck
                return True
        return False
