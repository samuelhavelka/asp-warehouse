import clingo
import subprocess
from clingo.symbol import Number
from clingo.control import Control

class Context:
    def inc(self, x):
        return Number(x.number + 1)
    def seq(self, x, y):
        return [x, y]

def on_model(m):
    print(m)

class Obj:
    def __init__(self, name):
        self.name = name


def parse_output(model):
    """
    Input:
        List of model symbols returned from clingo
    Output:
        Dict of warehouse objects
    """
    
    model = list(model)

    init = [s for s in model if "init(" in str(s)]
    node = [s for s in model if "node(" in str(s)]
    highway = [s for s in model if "highway(" in str(s)]
    pickingStation = [s for s in model if "pickingStation(" in str(s)]
    robot = [s for s in model if "robot(" in str(s)]
    shelf = [s for s in model if "shelf(" in str(s)]
    product = [s for s in model if "product(" in str(s)]
    order = [s for s in model if "order(" in str(s)]
    occurs = [s for s in model if "occurs(" in str(s)]
    numActions = [s for s in model if "numActions(" in str(s)]
    totalTime = [s for s in model if "totalTime(" in str(s)]

    output = {'init' : init,
              'node' : node,
              'highway' : highway,
              'pickingStation' : pickingStation,
              'robot' : robot,
              'shelf' : shelf,
              'product' : product,
              'order' : order,
              'occurs' : occurs,
              'numActions' : numActions,
              'totalTime' : totalTime,}

    return output


def main(program_path, instance_path):

    # read clingo main script
    with open(program_path, 'rt') as file:
        asp_file = file.read()
    
    # read initialization instance
    with open(instance_path, 'rt') as file:
        init_file = file.read()

    clingo_program = init_file + asp_file

    # initialize clingo control object
    ctl = Control()
    ctl.add("base", [], clingo_program)
    
    ctl.ground([("base", [])], context=Context())

    # run clingo scprit
    result = ctl.solve(yield_=True)

    models = []
    for r in result:
        m = r.symbols(atoms=True)
        models.append(list(m))
    best_model = models[-1]

    objects = parse_output(best_model)
    print(objects)


if __name__ == '__main__':
  # run warehouse optimization script with example instance 1
  main("warehouse.lp", "simpleInstances\inst1.lp")