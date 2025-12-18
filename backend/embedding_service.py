"""
Embedding Service Module
Provides unified interface for different embedding providers (Cohere, Sentence Transformers, etc.)
"""

import os
from abc import ABC, abstractmethod
from typing import List, Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import based on availability
try:
    import cohere
    COHERE_AVAILABLE = True
except ImportError:
    COHERE_AVAILABLE = False
    cohere = None

try:
    from sentence_transformers import SentenceTransformer
    SENTENCE_TRANSFORMERS_AVAILABLE = True
except ImportError:
    SENTENCE_TRANSFORMERS_AVAILABLE = False
    SentenceTransformer = None


class EmbeddingProvider(ABC):
    """Abstract base class for embedding providers"""
    
    @abstractmethod
    def embed(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """Generate embeddings for a list of texts"""
        pass
    
    @property
    @abstractmethod
    def embedding_dimensions(self) -> int:
        """Return the dimension of the embeddings produced"""
        pass


class CohereEmbeddingProvider(EmbeddingProvider):
    """Cohere embedding provider implementation"""
    
    def __init__(self, api_key: Optional[str] = None, model: str = "embed-english-v3.0"):
        if not COHERE_AVAILABLE:
            raise ImportError(
                "cohere-ai package is not installed. "
                "Install it with: pip install cohere-ai"
            )
        
        api_key = api_key or os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError(
                "COHERE_API_KEY not found in environment variables. "
                "Please set the COHERE_API_KEY environment variable."
            )
        
        self.client = cohere.Client(api_key)
        self.model = model
        # Cohere embed-english-v3.0 produces 1024-dimensional vectors by default
        # but can produce 384, 768, or 1024 based on truncate setting
        self._dimensions = 1024  # Default is 1024 for this model
    
    def embed(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings using Cohere API
        
        Args:
            texts: List of texts to embed
            input_type: Type of input (search_document, search_query, classification, etc.)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=input_type
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            raise Exception(f"Cohere API error: {str(e)}")
    
    @property
    def embedding_dimensions(self) -> int:
        return self._dimensions


class SentenceTransformersEmbeddingProvider(EmbeddingProvider):
    """Sentence Transformers embedding provider implementation"""
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        if not SENTENCE_TRANSFORMERS_AVAILABLE:
            raise ImportError(
                "sentence-transformers package is not installed. "
                "Install it with: pip install sentence-transformers"
            )
        
        self.model = SentenceTransformer(model_name)
        # all-MiniLM-L6-v2 produces 384-dimensional vectors
        self._dimensions = 384
    
    def embed(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings using Sentence Transformers (local)
        
        Args:
            texts: List of texts to embed
            input_type: Type of input (ignored for local models)
        """
        try:
            embeddings = self.model.encode(texts, convert_to_numpy=True)
            return [embedding.tolist() for embedding in embeddings]
        except Exception as e:
            raise Exception(f"Sentence Transformers error: {str(e)}")
    
    @property
    def embedding_dimensions(self) -> int:
        return self._dimensions


class EmbeddingService:
    """Service class that manages embedding providers and provides a unified interface"""
    
    def __init__(self, provider_type: str = "auto"):
        """
        Initialize the embedding service
        
        Args:
            provider_type: "cohere", "sentence_transformers", or "auto"
                          "auto" will use Cohere if available and API key is set, otherwise Sentence Transformers
        """
        self.provider_type = provider_type
        self.provider = self._initialize_provider()
    
    def _initialize_provider(self) -> EmbeddingProvider:
        """Initialize the appropriate embedding provider based on configuration"""
        
        if self.provider_type == "cohere":
            if not COHERE_AVAILABLE:
                raise ImportError("Cohere is not available. Install cohere-ai package.")
            return CohereEmbeddingProvider()
        
        elif self.provider_type == "sentence_transformers":
            if not SENTENCE_TRANSFORMERS_AVAILABLE:
                raise ImportError("Sentence Transformers is not available. Install sentence-transformers package.")
            return SentenceTransformersEmbeddingProvider()
        
        elif self.provider_type == "auto":
            # Try Cohere first if API key is available
            if COHERE_AVAILABLE and os.getenv("COHERE_API_KEY"):
                try:
                    return CohereEmbeddingProvider()
                except ValueError:
                    # Fall back to Sentence Transformers if Cohere key is not valid
                    pass
            
            # Fall back to Sentence Transformers
            if SENTENCE_TRANSFORMERS_AVAILABLE:
                print("⚠️  Using Sentence Transformers (local embeddings) - no Cohere API key found")
                return SentenceTransformersEmbeddingProvider()
            else:
                raise ImportError(
                    "No embedding providers available. Install either cohere-ai or sentence-transformers package."
                )
        
        else:
            raise ValueError(f"Invalid provider type: {self.provider_type}. Use 'cohere', 'sentence_transformers', or 'auto'.")
    
    def embed(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using the configured provider
        
        Args:
            texts: List of texts to embed
            input_type: Type of input (used for Cohere, ignored for Sentence Transformers)
        """
        return self.provider.embed(texts, input_type)
    
    def embed_single(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text
        
        Args:
            text: Single text to embed
            input_type: Type of input (used for Cohere, ignored for Sentence Transformers)
        """
        embeddings = self.embed([text], input_type)
        return embeddings[0] if embeddings else []
    
    @property
    def embedding_dimensions(self) -> int:
        """Return the dimension of embeddings produced by the current provider"""
        return self.provider.embedding_dimensions
    
    @property
    def current_provider_name(self) -> str:
        """Return the name of the current embedding provider"""
        if isinstance(self.provider, CohereEmbeddingProvider):
            return "cohere"
        elif isinstance(self.provider, SentenceTransformersEmbeddingProvider):
            return "sentence_transformers"
        else:
            return "unknown"


# Global instance for easy access
embedding_service = EmbeddingService(provider_type="auto")


def get_embedding_service(provider_type: str = "auto") -> EmbeddingService:
    """Get an embedding service instance"""
    return EmbeddingService(provider_type)


if __name__ == "__main__":
    # Test the embedding service
    print("Testing Embedding Service...")
    
    test_texts = [
        "Artificial intelligence is transforming robotics",
        "Humanoid robots combine sensors and actuators",
        "Cohere provides high-quality text embeddings"
    ]
    
    # Use the global instance
    service = embedding_service
    
    print(f"Using provider: {service.current_provider_name}")
    print(f"Embedding dimensions: {service.embedding_dimensions}")
    
    try:
        embeddings = service.embed(test_texts)
        print(f"Generated {len(embeddings)} embeddings")
        print(f"First embedding length: {len(embeddings[0])}")
        print("✅ Embedding service test successful!")
    except Exception as e:
        print(f"❌ Error during test: {str(e)}")