import numpy as np
from scipy.stats import norm
from scipy.special import logsumexp
import scipy
import math
import copy

rotation_noise = 0.05
translation_noise = 0.035
sensor_noise = 0.1

class Particle:
    def __init__(self, x, y, theta, ln_p):
        self.x = x
        self.y = y
        self.theta = theta
        self.ln_p = ln_p
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}, prob: {self.ln_p}"
class ParticleFilter:
    def __init__(self, map, num_particles, init_pos):
        assert len(init_pos) == 3, "the starting position must be a tuple of three"
        self._particles = []
        self.map = map
        self.eff = 0
        ln_p = 1/num_particles
        for _ in range(num_particles):
            x = init_pos[0]
            y = init_pos[1]
            theta = init_pos[2]
            self._particles.append(Particle(x, y, theta, ln_p))

    def move_by(self, x, y, theta):
        distance = math.sqrt(x**2 + y**2)
        for p in self._particles:
            #print(p)
            random_loc_noise = np.random.normal(0, translation_noise)
            random_theta_noise = np.random.normal(0, rotation_noise)
            p.theta =  p.theta + theta + random_theta_noise
            new_distance = distance + random_loc_noise
            p.x = p.x + new_distance * math.cos(p.theta)
            p.y = p.y + new_distance * math.sin(p.theta)
            #print(p)
            #print()
    def measure(self, sonar_reading, servo_angle_in_rad):
        log_prob = []
        for p in self._particles:
            location = self.map.closest_distance((p.x, p.y), p.theta + servo_angle_in_rad)
            if location == None:
                log_prob.append(np.NINF) #treat as lowest probability possible
            else:
                likelihood = norm(location, sensor_noise).pdf(sonar_reading)
                prior = p.ln_p
                log_prob.append(math.log(likelihood) + math.log(prior))

        #normalize
        sum = logsumexp(log_prob)
        check = 0
        eff = 0
        for i in range(len(self._particles)):
            self._particles[i].ln_p = math.exp(log_prob[i] - sum)
            if(self._particles[i].ln_p == 0):
                self._particles[i].ln_p = 1.0e-30

            check += self._particles[i].ln_p
            eff += self._particles[i].ln_p**2
            #print(self._particles[i])
        #sanity check
        #print("sum of probability: ", check)
        self.eff = 1/eff
        #print("effective num", self.eff)
        #resample
        if self.eff < len(self._particles)*0.9:
            new_particles = []
            for i in range(len(self._particles)):
                sample = np.random.choice(self._particles, p = [particle.ln_p for particle in self._particles])
                new_particles.append(copy.deepcopy(sample))
                #print(sample)
            self._particles = new_particles

        #print([i.ln_p for i in sorted(self._particles, key = lambda x: x.ln_p)])
    def get_estimate(self):
        #return np.argmax(np.array([p.ln_p for p in self._particles])))
        xs, ys, thetas, probs = ([],[],[],[])
        for p in self._particles:
            xs.append(p.x)
            ys.append(p.y)
            thetas.append(p.theta)
            probs.append(p.ln_p)
        x = np.average(xs, weights = probs)
        y = np.average(ys, weights = probs)
        theta = np.average(thetas, weights = probs)

        return (x,y,theta)

    def get_particles(self):
        result = []
        for p in self._particles:
            result.extend([p.x, p.y, 0.1, p.theta])
        return result

    def get_best_prob_particle(self):
        p = sorted(self._particles, key = lambda x: x.ln_p)[-1]
        return (p.x, p.y, p.theta)

    def get_converge(self):
        #calculate the error
        avg = self.get_estimate()
        error = 0
        for i in self._particles:
            error += math.sqrt((i.x - avg[0])**2 + (i.y - avg[1])**2)
        error /= len(self._particles)
        return error

    def get_effectiness(self):
        return self.eff

    def need_update(self, it, path_len):
        con = self.get_converge()
        eff = self.get_effectiness()
        #confidence of the particle filtering
        #how converge is the particle filter
        print("error", con, "eff", eff)
        #return (con < 0.08 and eff < 150) or (con < 0.02)
        factor = 0.13 /((path_len ** 2) / 4)
        eq = -it * factor *(it - path_len)
        eq = min(eq, 0.090)
        print(eq)

        if it < 3 or it > (path_len - 3):
            return False
        else:
            return eff < len(self._particles) * 0.45
