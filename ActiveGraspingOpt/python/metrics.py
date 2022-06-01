from pygrasp.pygrasp import GraspResult

class GraspMetric(object):

    def get_data(self, res: GraspResult) -> "tuple[list, str, list]":
        """
        return tuple with:
        - metrics: [(name, value)]
        - error: error_name
        - metadata: [(name, value)]
        """
        metrics = self.get_metrics(res)
        fails = self.get_failures(res)
        metadata = self.get_metadata(res)

        return metrics, fails, metadata
    
    def get_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        raise Exception("This is an abstract class")

    def get_failures(self, res: GraspResult) -> str:
        raise Exception("This is an abstract class")

    def get_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        raise Exception("This is an abstract class")

class GramacyMetric(GraspMetric):
    
    def get_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("outcome", res.measure)]

    def get_failures(self, res: GraspResult) -> str:
        return ""

    def get_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return []

class ForceClosure(GraspMetric):
    
    def get_metrics(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("outcome", res.measure)]

    def get_failures(self, res: GraspResult) -> str:
        return res.error

    def get_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return [("volume", res.volume), ("force_closure", res.force_closure)]

class ForceClosureIK(ForceClosure):
    def get_metadata(self, res: GraspResult) -> "list[tuple[str, any]]":
        return  [("volume", res.volume),
                ("force_closure", res.force_closure),
                ("time(ms)", res.time),
                ("position_error(mm)", res.pos_error),
                ("orientation_error(degrees)", res.ori_error)]
    
