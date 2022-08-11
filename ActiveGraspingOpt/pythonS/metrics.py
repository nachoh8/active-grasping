from pygrasp.pygrasp import GraspResult

MTYPE_GRAMACY = 0
MTYPE_FC = 1
MTYPE_FC_IK = 2
MTYPE_FC_T_IK = 3

class GraspMetric(object):

    def get_data_log(self, res: GraspResult) -> dict:
        """
        return data to local log
        """
        raise Exception("This is an abstract class")
    
    def get_bayesopt_metric(self, res: GraspResult) -> float:
        """
        return the metric fot bayesopt optimizer
        """
        raise Exception("This is an abstract class")

    def get_sigopt_data(self, res: GraspResult) -> "tuple[list, str, list]":
        """
        return tuple with:
        - metrics: [(name, value)]
        - error: error_name
        - metadata: [(name, value)]
        """
        metrics = self.get_sigopt_metrics(res)
        fails = self.get_sigopt_failures(res)
        metadata = self.get_sigopt_metadata(res)

        return metrics, fails, metadata
    
    def get_sigopt_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        raise Exception("This is an abstract class")

    def get_sigopt_failures(self, res: GraspResult) -> str:
        raise Exception("This is an abstract class")

    def get_sigopt_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        raise Exception("This is an abstract class")

class GramacyMetric(GraspMetric):
    
    def get_data_log(self, res: GraspResult) -> dict:
        data = {}
        data["metrics"] = {"outcome": res.measure}

        return data
    
    def get_bayesopt_metric(self, res: GraspResult) -> float:
        return -res.measure

    def get_sigopt_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("outcome", res.measure)]

    def get_sigopt_failures(self, res: GraspResult) -> str:
        return ""

    def get_sigopt_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return []

class ForceClosure(GraspMetric):
    def get_data_log(self, res: GraspResult) -> dict:
        data = {}
        data["metrics"] = {"outcome": res.measure, "volume": res.volume, "force_closure": res.force_closure}

        if res.error != "":
            data["error"] = res.error

        return data
    
    def get_bayesopt_metric(self, res: GraspResult) -> float:
        return -res.measure

    def get_sigopt_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("outcome", res.measure)]

    def get_sigopt_failures(self, res: GraspResult) -> str:
        return res.error

    def get_sigopt_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("volume", res.volume), ("force_closure", res.force_closure)]

class ForceClosureIK(ForceClosure):
    def get_data_log(self, res: GraspResult) -> dict:
        data = {}
        data["metrics"] = {"outcome": res.measure, "volume": res.volume, "force_closure": res.force_closure}
        data["others"] = {"time": res.time, "position_error": res.pos_error, "orientation_error": res.ori_error}

        if res.error != "":
            data["error"] = res.error

        return data
    
    def get_sigopt_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return  [("volume", res.volume),
                ("force_closure", res.force_closure),
                ("time(ms)", res.time),
                ("position_error(mm)", res.pos_error),
                ("orientation_error(degrees)", res.ori_error)]

class ForceClosureTimeIK(ForceClosureIK):
    def get_bayesopt_metric(self, res: GraspResult) -> float:
        raise Exception("This metric is not compatible with bayesopt")

    def get_sigopt_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("outcome", res.measure), ("time", res.time)]