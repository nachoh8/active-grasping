import argparse
import os

import sigopt

from pygrasp import TestGramacyExecutor, GraspResult

PROJECT_NAME = "test-gramacy-sigopt"
NUM_RUNS = 100

def executor(model: TestGramacyExecutor, run: sigopt.run_context.RunContext):
    run.log_model("Test Gramacy optimization")
    
    query = [run.params.x1, run.params.x2]
    res: GraspResult = model.executeQueryGrasp(query)
    
    print("Query:", query, "-> Outcome:", res.measure)

    run.log_metric("outcome", res.measure)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test Gramacy SigOpt Experiment')
    parser.add_argument("--dev", dest="use_dev_token", action='store_true', default=True)
    parser.add_argument("--prod", dest="use_dev_token", action='store_false')
    
    args = parser.parse_args()
    print("Project: " + PROJECT_NAME)
    if args.use_dev_token:
        print("Mode: dev")
        os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_DEV_TOKEN"]
    else:
        print("Mode: production")
        os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_PROD_TOKEN"]

    gramacy = TestGramacyExecutor()

    sigopt.set_project(PROJECT_NAME)

    experiment = sigopt.create_experiment(
        name="Gramacy optimization",
        type="offline",
        parameters=[
            dict(name="x1", type="double", bounds=dict(min=0.0001, max=1.0000), prior=dict(mean=0.5, name="normal", scale=0.2)),
            dict(name="x2", type="double", bounds=dict(min=0.0001, max=1.0000), prior=dict(mean=0.5, name="normal", scale=0.2)),
        ],
        metrics=[dict(name="outcome", strategy="optimize", objective="maximize")],
        parallel_bandwidth=1,
        budget=NUM_RUNS,
    )

    print("Begin experiment")
    print("-------------------------------")
    it = 1
    for run in experiment.loop():
        print("Run " + str(it) + "/" + str(NUM_RUNS))
        with run:
            executor(gramacy, run)
        
        it += 1
    
    print("-------------------------------")
    print("End experiment")

    
