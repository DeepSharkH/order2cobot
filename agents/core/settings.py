"""Application settings loaded from environment variables and .env."""

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="O2C_", env_file=".env", extra="ignore")

    log_level: str = "INFO"
    agent_timeout_s: float = 30.0
