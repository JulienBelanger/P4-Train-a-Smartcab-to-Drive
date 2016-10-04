import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
from collections import defaultdict

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    gamma = .5
    alphaDecay = 5
    epsilonDecay = 0
    badTrip = 0
    badMove = 0
    optimalMove = 0
    # defaultdict idea taken from aalattar on https://discussions.udacity.com/t/for-those-who-are-completely-new-and-completely-lost/168240
    qTable = defaultdict(int)

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint

        # TODO: Initialize any additional variables here
        LearningAgent.qTable[(None, None)]
        self.counter = 0
        self.endCounter = 0
        self.badTripCounter = 0
        self.badMoveCounter = 0
        self.optimalMoveCounter = 0
        self.Q = 0
        self.memR = [0]
        self.memStaAct = [(None,None)]


    def reset(self, destination=None):
        self.planner.route_to(destination)

        # TODO: Prepare for a new trip; reset any variables here, if required
        self.Q = 0
        self.memR = [0]
        self.memStaAct = [(None, None)]


    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)

        deadline = self.env.get_deadline(self)
        self.alpha = float(LearningAgent.alphaDecay) / (float(LearningAgent.alphaDecay) + float(self.counter))

        # TODO: Update state
        self.state = (inputs['light'], self.next_waypoint, inputs['oncoming'], inputs['left'])


        # TODO: Select action according to your policy
        b = 0 # debug

        if Simulator.opTrial > 100 and random.random() < (1 - LearningAgent.epsilonDecay/Simulator.opTrial):
            a = random.choice([None, 'forward', 'left', 'right'])
            for key1, value1 in LearningAgent.qTable.iteritems():
                for key2, value2 in LearningAgent.qTable.iteritems():
                    if key1 == key2:
                        pass
                    elif key1[0] == self.state and key2[0] == self.state:
                        if value1 == value2:
                            a = random.choice([key1[1], key2[1]])
                        elif value1 > value2:
                            a = key1[1]
                            b = 1 # debug
                        else:
                            a = key2[1]
                            b = 1 # debug
            action = a
            if b == 0: print('Random Move') # debug
            else : print('Q Move') # debug

        else :
            print('Random Move')
            action = random.choice([None, 'forward', 'left', 'right'])


        # Execute action and get reward and upload memories

        # The bonus makes no sense in terms of this problem
        reward = self.env.act(self, action)
        if reward > 2: reward = reward - 10
        self.memR.append(reward)
        self.memStaAct.append((self.state,action))
        LearningAgent.qTable[self.memStaAct[-1]] = self.memR[-1]

        # TODO: Learn policy based on state, action, reward
        # Static formula for Q update taken on https://discussions.udacity.com/t/what-is-basic-q-learning-vs-enhanced/165072/2
        for i in range(len(self.memR)-2,-1,-1):
            LearningAgent.qTable[self.memStaAct[i]] = LearningAgent.qTable[self.memStaAct[i]]*(1-self.alpha) + self.alpha*(self.memR[i] + LearningAgent.gamma*LearningAgent.qTable[self.memStaAct[i+1]])
        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, self.memR[-1])  # [debug]

        # TODO: Learning Quantities
        if Simulator.opTrial > 100:
            self.endCounter += 1
            if deadline == 0:
                self.badTripCounter += 1
            if reward == -1:
                self.badMoveCounter += 1
            if reward == 2:
                self.optimalMove += 1
            LearningAgent.badTrip = float(self.badTripCounter)/(Simulator.opTrial - 100) * 100.00
            LearningAgent.badMove = float(self.badMoveCounter)/(self.endCounter) * 100.00
            LearningAgent.optimalMove = float(self.optimalMove)/(self.endCounter) * 100.00

def run(p):
    """Run the agent for a finite number of trials."""

    #########################################
    LearningAgent.gamma = p[0]
    LearningAgent.alphaDecay = p[1]
    LearningAgent.epsilon = p[2]
    #########################################

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # set agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0)  # reduce update_delay to speed up simulation
    sim.run(n_trials=201)  # press Esc or close pygame window to quit

    return [LearningAgent.badTrip,LearningAgent.badMove,LearningAgent.optimalMove]

if __name__ == '__main__':
    temp = run([.05,50,5])
    print "Percentage of good trips : {} %".format(100.00 - temp[0])
    print "Percentage of bad moves: {} %".format(temp[1])
    print "Percentage of optimal moves : {} %".format(temp[2])