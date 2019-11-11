import random


class ParamRandomizer:
    # Confidence is manually selected by the designer. Difficulty should start at zero during training and end at 1.
    # 1 means that the value used by the simulation is random over the whole confidence interval.
    def __init__(self, default, confidence_95_pc, difficulty = 0):
        self.__default = default
        self.difficulty = self.set_difficulty(difficulty)
        self.value = default
        self.confidence_95_pc = confidence_95_pc
        self.randomize(difficulty)


    def set_difficulty(self, difficulty):
        if difficulty < 0:
            raise ValueError("The difficulty cannot be negative")
        if difficulty > 1:
            raise ValueError("The difficulty cannot be over 1")
        return difficulty

    def randomize(self, difficulty):
        self.difficulty = self.set_difficulty(difficulty)

        min_pick = self.__default - self.confidence_95_pc * difficulty
        max_pick = self.__default + self.confidence_95_pc * difficulty
        self.value = random.uniform(min_pick, max_pick)