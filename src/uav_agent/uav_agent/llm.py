#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

import dotenv
from azure.identity import ClientSecretCredential, get_bearer_token_provider
from langchain_openai import AzureChatOpenAI
from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama


def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    # APIM_SUBSCRIPTION_KEY = get_env_variable("APIM_SUBSCRIPTION_KEY")
    # default_headers = {}
    # if APIM_SUBSCRIPTION_KEY is not None:
    #     # only set this if the APIM API requires a subscription...
    #     default_headers["Ocp-Apim-Subscription-Key"] = APIM_SUBSCRIPTION_KEY

    # Set up authority and credentials for Azure authentication
    # credential = ClientSecretCredential(
    #     tenant_id=get_env_variable("AZURE_TENANT_ID"),
    #     client_id=get_env_variable("AZURE_CLIENT_ID"),
    #     client_secret=get_env_variable("AZURE_CLIENT_SECRET"),
    #     authority="https://login.microsoftonline.com",
    # )

    # token_provider = get_bearer_token_provider(
    #     credential, "https://cognitiveservices.azure.com/.default"
    # )

    # llm = AzureChatOpenAI(
    #     azure_deployment=get_env_variable("DEPLOYMENT_ID"),
    #     azure_ad_token_provider=token_provider,
    #     openai_api_type="azure_ad",
    #     api_version=get_env_variable("API_VERSION"),
    #     azure_endpoint=get_env_variable("API_ENDPOINT"),
    #     default_headers=default_headers,
    #     streaming=streaming,
    # )
    
    # openai_llm = ChatOpenAI(
    #     model_name="gpt-4o-mini",  # or your preferred model
    #     temperature=0,
    #     max_tokens=None,
    #     timeout=None,
    #     max_retries=2,
    #     openai_api_key=os.getenv("OPENAI_API_KEY"),  # Using environment variable
    # )

    ollama_llm = ChatOllama(
        # model="qwen2:7b",  # or your preferred model
        model="llama3.1:8b",  # or your preferred model
        base_url="http://localhost:11434",
        temperature=0,
        num_ctx=8192,  # adjust based on your model's context window
    )


    # return llm
    return ollama_llm


def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name)
    if value is None:
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value
