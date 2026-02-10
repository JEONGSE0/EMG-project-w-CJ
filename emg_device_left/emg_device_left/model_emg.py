# core/model_runner.py
import joblib
import numpy as np


class ModelRunner:
    """
    Loads a saved sklearn pipeline/model from a file path and runs inference.

    Supports:
      - joblib file containing a Pipeline/estimator directly
      - joblib file containing a dict with keys like:
        {"model": pipeline}, {"pipeline": ...}, {"clf": ...}, {"estimator": ...}
    """

    def __init__(self, model_path: str):
        self.model_path = model_path
        self.pipeline = None

    def load(self):
        obj = joblib.load(self.model_path)
        self.pipeline = self._unwrap(obj)

        # sanity check
        if not hasattr(self.pipeline, "predict_proba"):
            raise ValueError(
                "Loaded model does not support predict_proba(). "
                "Please train SVM with probability=True and save the pipeline."
            )
        
        self.classes_ = None
        if hasattr(self.pipeline, "classes_"):
            self.classes_ = self.pipeline.classes_
        else:
            try:
                last = list(self.pipeline.named_steps.values())[-1]
                if hasattr(last, "classes_"):
                    self.classes_ = last.classes_
            except Exception:
                pass

        print("Loaded classes_:", self.classes_)
        
        

    @staticmethod
    def _unwrap(obj):
        # If dict, try common keys
        if isinstance(obj, dict):
            for k in ("model", "pipeline", "clf", "estimator"):
                if k in obj:
                    return obj[k]
            raise ValueError(
                f"Loaded object is dict, but no model key found. keys={list(obj.keys())}"
            )
        return obj

    def predict_proba(self, feat_vec: np.ndarray) -> np.ndarray:
        """
        feat_vec: (D,)
        returns: (C,)
        """
        if self.pipeline is None:
            self.load()
        X = np.asarray(feat_vec, dtype=np.float64).reshape(1, -1)
        proba = self.pipeline.predict_proba(X)[0]
        return np.asarray(proba, dtype=np.float64)

    def predict_class(self, feat_vec: np.ndarray) -> int:
        proba = self.predict_proba(feat_vec)
        return int(np.argmax(proba))
