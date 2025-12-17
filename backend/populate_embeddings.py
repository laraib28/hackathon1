"""
Populate Qdrant Collection with Document Embeddings
Loads markdown files, chunks them, generates embeddings, and inserts into Qdrant
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Tuple
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

# Import embedding service
from embedding_service import get_embedding_service, EmbeddingService

# Configuration
COLLECTION_NAME = "humanoid_robotics_docs"
DOCS_FOLDER = "../humanoid-robotics-book/docs"
CHUNK_SIZE = 500  # Characters per chunk
CHUNK_OVERLAP = 100  # Overlap between chunks
BATCH_SIZE = 10  # Process embeddings in batches


class DocumentChunker:
    """Handles document chunking with overlap"""

    @staticmethod
    def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
        """
        Split text into overlapping chunks

        Args:
            text: Text to chunk
            chunk_size: Maximum characters per chunk
            overlap: Number of overlapping characters between chunks

        Returns:
            List of text chunks
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # Try to break at sentence boundary
            if end < len(text):
                # Look for sentence endings near the chunk boundary
                sentence_end = text.rfind('.', start, end)
                if sentence_end > start + chunk_size // 2:  # Only use if reasonably close to end
                    end = sentence_end + 1

            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)

            # Move start position with overlap
            start = end - overlap if end < len(text) else end

        return chunks

    @staticmethod
    def clean_markdown(text: str) -> str:
        """Remove markdown syntax for cleaner embeddings"""
        # Remove code blocks
        text = re.sub(r'```[\s\S]*?```', '', text)
        # Remove inline code
        text = re.sub(r'`[^`]+`', '', text)
        # Remove markdown links but keep text
        text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)
        # Remove headers markers but keep text
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
        # Remove emphasis markers
        text = re.sub(r'[*_]{1,2}([^*_]+)[*_]{1,2}', r'\1', text)
        # Remove extra whitespace
        text = re.sub(r'\n{3,}', '\n\n', text)
        return text.strip()


class QdrantPopulator:
    """Handles Qdrant collection population"""

    def __init__(self, embedding_service: EmbeddingService):
        self.qdrant = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.embedding_service = embedding_service
        self.collection_name = COLLECTION_NAME
        self.vector_size = embedding_service.embedding_dimensions

    def create_collection_if_not_exists(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.qdrant.get_collection(self.collection_name)
            print(f"✅ Collection '{self.collection_name}' already exists")
        except Exception:
            # Create collection
            print(f"📦 Creating collection '{self.collection_name}'...")
            self.qdrant.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"✅ Collection created with vector size: {self.vector_size}")

    def get_collection_stats(self) -> Dict:
        """Get collection statistics"""
        try:
            collection_info = self.qdrant.get_collection(self.collection_name)
            return {
                "vectors_count": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "status": collection_info.status
            }
        except Exception as e:
            return {"error": str(e)}

    def upsert_documents(self, documents: List[Dict]):
        """
        Upsert documents into Qdrant

        Args:
            documents: List of document dictionaries with 'content', 'url', 'metadata'
        """
        if not documents:
            print("⚠️  No documents to upsert")
            return

        print(f"\n📝 Processing {len(documents)} document chunks...")

        # Process in batches
        for i in range(0, len(documents), BATCH_SIZE):
            batch = documents[i:i + BATCH_SIZE]

            # Extract texts for embedding
            texts = [doc['content'] for doc in batch]

            # Generate embeddings
            print(f"🔄 Generating embeddings for batch {i//BATCH_SIZE + 1} ({len(texts)} chunks)...")
            embeddings = self.embedding_service.embed(texts, input_type="search_document")

            # Prepare points for Qdrant
            points = []
            for doc, embedding in zip(batch, embeddings):
                point_id = str(uuid.uuid4())

                points.append(
                    models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "content": doc['content'],
                            "url": doc['url'],
                            "source": doc['metadata']['source'],
                            "chunk_index": doc['metadata']['chunk_index'],
                            "total_chunks": doc['metadata']['total_chunks'],
                            "indexed_at": datetime.utcnow().isoformat()
                        }
                    )
                )

            # Upsert to Qdrant
            self.qdrant.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"✅ Batch {i//BATCH_SIZE + 1} inserted ({len(points)} points)")


def load_markdown_files(docs_folder: str) -> List[Tuple[str, str]]:
    """
    Load all markdown files from the docs folder

    Returns:
        List of (file_path, content) tuples
    """
    docs_path = Path(docs_folder)

    if not docs_path.exists():
        print(f"❌ Docs folder not found: {docs_folder}")
        return []

    markdown_files = []

    # Find all .md files
    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # Get relative path for URL
                relative_path = md_file.relative_to(docs_path.parent)
                markdown_files.append((str(relative_path), content))

        except Exception as e:
            print(f"⚠️  Error reading {md_file}: {e}")

    return markdown_files


def prepare_documents(markdown_files: List[Tuple[str, str]]) -> List[Dict]:
    """
    Prepare documents for embedding: chunk, clean, and add metadata

    Returns:
        List of document dictionaries ready for embedding
    """
    chunker = DocumentChunker()
    documents = []

    for file_path, content in markdown_files:
        # Clean markdown
        cleaned_content = chunker.clean_markdown(content)

        # Skip if content is too short
        if len(cleaned_content) < 50:
            continue

        # Chunk the content
        chunks = chunker.chunk_text(cleaned_content)

        # Create document entries
        for idx, chunk in enumerate(chunks):
            doc = {
                'content': chunk,
                'url': f"/docs/{file_path}",
                'metadata': {
                    'source': file_path,
                    'chunk_index': idx,
                    'total_chunks': len(chunks)
                }
            }
            documents.append(doc)

    return documents


def main():
    """Main execution function"""
    print("=" * 70)
    print("🚀 Qdrant Collection Population Script")
    print("=" * 70)

    # Initialize embedding service
    print("\n1️⃣  Initializing embedding service...")
    embedding_service = get_embedding_service(provider_type="auto")
    print(f"✅ Using: {embedding_service.current_provider_name}")
    print(f"✅ Embedding dimensions: {embedding_service.embedding_dimensions}")

    # Initialize Qdrant populator
    print("\n2️⃣  Connecting to Qdrant...")
    populator = QdrantPopulator(embedding_service)

    # Check current stats
    print("\n3️⃣  Checking collection status...")
    stats = populator.get_collection_stats()
    if 'error' in stats:
        print(f"⚠️  Collection doesn't exist yet: {stats['error']}")
    else:
        print(f"📊 Current vectors: {stats.get('vectors_count', 0)}")
        print(f"📊 Vector size: {stats.get('vector_size', 0)}")
        print(f"📊 Status: {stats.get('status', 'unknown')}")

    # Create collection if needed
    print("\n4️⃣  Ensuring collection exists...")
    populator.create_collection_if_not_exists()

    # Load markdown files
    print(f"\n5️⃣  Loading markdown files from: {DOCS_FOLDER}")
    markdown_files = load_markdown_files(DOCS_FOLDER)
    print(f"✅ Found {len(markdown_files)} markdown files")

    if not markdown_files:
        print("❌ No markdown files found. Exiting.")
        return

    # Prepare documents
    print("\n6️⃣  Preparing documents (chunking and cleaning)...")
    documents = prepare_documents(markdown_files)
    print(f"✅ Prepared {len(documents)} document chunks")

    # Show sample
    if documents:
        print("\n📄 Sample chunk:")
        print(f"   Content: {documents[0]['content'][:150]}...")
        print(f"   URL: {documents[0]['url']}")
        print(f"   Chunk: {documents[0]['metadata']['chunk_index'] + 1}/{documents[0]['metadata']['total_chunks']}")

    # Upsert documents
    print("\n7️⃣  Upserting documents to Qdrant...")
    populator.upsert_documents(documents)

    # Final stats
    print("\n8️⃣  Final collection statistics:")
    final_stats = populator.get_collection_stats()
    print(f"📊 Total vectors: {final_stats.get('vectors_count', 0)}")
    print(f"📊 Vector size: {final_stats.get('vector_size', 0)}")
    print(f"📊 Status: {final_stats.get('status', 'unknown')}")

    print("\n" + "=" * 70)
    print("✅ COMPLETE! Embeddings successfully populated in Qdrant")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
