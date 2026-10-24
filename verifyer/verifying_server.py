"""Picture + statement -> JSON boolean, served by one reusable MiniCPM-V model."""

from contextlib import asynccontextmanager
from threading import Lock
from typing import Annotated

import torch
import uvicorn
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from PIL import Image, UnidentifiedImageError
from transformers import AutoModelForImageTextToText, AutoProcessor


class VerifyingServer:
    def __init__(
        self,
        model_id="openbmb/MiniCPM-V-4.7-1B",
        device=None,
        downsample_mode="4x",
    ):
        self.model_id = model_id
        self.device = device or (
            "cuda" if torch.cuda.is_available()
            else "mps" if torch.backends.mps.is_available()
            else "cpu"
        )
        if downsample_mode not in {"4x", "16x"}:
            raise ValueError("downsample_mode must be '4x' or '16x'")
        self.downsample_mode = downsample_mode
        self.model = None
        self.processor = None
        self._lock = Lock()
        self.app = FastAPI(title="Picture statement verifier", lifespan=self._lifespan)
        self.app.add_api_route("/verify", self.verify, methods=["POST"], response_model=bool)
        self.app.add_api_route("/health", self.health, methods=["GET"])

    def load_model(self):
        """Load once at server startup, before accepting requests."""
        if self.model is not None:
            return
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForImageTextToText.from_pretrained(
            self.model_id,
            dtype=torch.bfloat16 if self.device.startswith("cuda") else torch.float32,
            attn_implementation="sdpa",
        ).to(self.device).eval()

    @asynccontextmanager
    async def _lifespan(self, app):
        self.load_model()
        try:
            yield
        finally:
            self.model = None
            self.processor = None

    @torch.inference_mode()
    def predict(self, image, statement):
        """Return a Python bool. Serialize calls to the shared GPU model."""
        with self._lock:
            if self.model is None or self.processor is None:
                raise HTTPException(status_code=503, detail="Model is not loaded")
            messages = [{
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": (
                        f"Statement: {statement}\n"
                        "Does the picture support the statement? "
                        "Answer yes only if visible evidence supports it; otherwise answer no. "
                        "Reply with exactly one word: yes or no."
                    )},
                ],
            }]
            inputs = self.processor.apply_chat_template(
                messages,
                tokenize=True,
                add_generation_prompt=True,
                return_dict=True,
                return_tensors="pt",
                enable_thinking=False,
                processor_kwargs={
                    "downsample_mode": self.downsample_mode,
                    "max_slice_nums": 36,
                },
            ).to(self.model.device)
            generated_ids = self.model.generate(
                **inputs,
                downsample_mode=self.downsample_mode,
                do_sample=False,
                max_new_tokens=8,
            )
            answer = self.processor.decode(
                generated_ids[0, inputs["input_ids"].shape[-1]:],
                skip_special_tokens=True,
            ).strip().lower().rstrip(".!?")
            if answer not in {"yes", "no"}:
                raise HTTPException(status_code=502, detail="Model did not return a yes/no answer")
            return answer == "yes"

    def verify(
        self,
        picture: Annotated[UploadFile, File(description="Picture to evaluate")],
        statement: Annotated[str, Form(min_length=1, max_length=4096)],
    ) -> bool:
        """POST multipart form data with a picture file and a statement."""
        statement = statement.strip()
        if not statement:
            raise HTTPException(status_code=422, detail="Statement must not be blank")
        # Decode the uploaded image directly; no client-supplied paths are opened.
        try:
            with Image.open(picture.file) as uploaded:
                image = uploaded.convert("RGB")
        except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
            raise HTTPException(status_code=400, detail="Picture must be a valid image")
        try:
            return self.predict(image, statement)
        finally:
            image.close()

    def health(self):
        return {"ready": self.model is not None, "model": self.model_id, "device": self.device}

    def run(self, host="127.0.0.1", port=8000):
        """Start the HTTP server. Keep one worker to share one model instance."""
        uvicorn.run(self.app, host=host, port=port)


if __name__ == "__main__":
    VerifyingServer().run()
