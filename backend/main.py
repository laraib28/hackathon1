"""
Embedding Generation Script for Humanoid Robotics Documentation
Fetches content from sitemap, extracts text, generates embeddings, and stores in Qdrant
Using Sentence Transformers (100% FREE - No API needed!)
"""

import requests
import xml.etree.ElementTree as ET
import trafilatura as tfl
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from sentence_transformers import SentenceTransformer
import time
import os
from typing import List, Dict
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
SITEMAP_URL = "https://hackathon1-r5lc.vercel.app/sitemap.xml"
COLLECTION_NAME = "humanoid_robotics_docs"

# Initialize Sentence Transformer (runs locally - no API needed!)
print("📥 Loading embedding model (first time may take a moment to download)...")
model = SentenceTransformer('all-MiniLM-L6-v2')  # 384 dimensions, fast and efficient
EMBED_DIM = 384

print("✅ Model loaded successfully!")

# Initialize Qdrant client
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

def get_all_urls(sitemap_url: str) -> List[str]:
    """
    Extract all URLs from sitemap.xml
    """
    try:
        xml = requests.get(sitemap_url).text
        root = ET.fromstring(xml)

        # Handle namespace
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = []

        # Extract all URLs
        for url_elem in root.findall('.//ns:url', namespace):
            loc_tag = url_elem.find('ns:loc', namespace)
            if loc_tag is not None and loc_tag.text:
                urls.append(loc_tag.text)

        print(f"\n✅ FOUND {len(urls)} URLS:")
        for u in urls:
            print(f"  - {u}")

        return urls
    except Exception as e:
        print(f"❌ Error fetching sitemap: {e}")
        return []

def extract_text_from_url(url: str) -> str:
    """
    Extract clean text content from a URL
    """
    try:
        html = requests.get(url).text
        text = tfl.extract(html)

        if not text:
            print(f"⚠️  [WARNING] No text extracted from: {url}")
            return ""

        return text
    except Exception as e:
        print(f"❌ Error extracting text from {url}: {e}")
        return ""

def chunk_text(text: str, chunk_size: int = 500) -> List[str]:
    """
    Split text into smaller chunks
    """
    # Remove extra whitespace
    text = ' '.join(text.split())

    # Split into sentences
    sentences = text.split('. ')
    chunks = []
    current_chunk = ''

    for sentence in sentences:
        if len(current_chunk) + len(sentence) > chunk_size and current_chunk:
            chunks.append(current_chunk.strip() + '.')
            current_chunk = sentence
        else:
            current_chunk += (' ' if current_chunk else '') + sentence

    if current_chunk:
        chunks.append(current_chunk.strip())

    # Filter out very small chunks
    return [c for c in chunks if len(c) > 50]

def create_collection():
    """
    Create or recreate Qdrant collection
    """
    try:
        # Delete existing collection if it exists
        try:
            qdrant.delete_collection(COLLECTION_NAME)
            print(f"🗑️  Deleted existing collection: {COLLECTION_NAME}")
        except:
            print(f"ℹ️  No existing collection to delete")

        # Create new collection with Sentence Transformer dimensions
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBED_DIM,  # all-MiniLM-L6-v2 dimension
                distance=Distance.COSINE
            )
        )
        print(f"✅ Created collection: {COLLECTION_NAME}\n")
    except Exception as e:
        print(f"❌ Error creating collection: {e}")
        raise

def generate_embeddings(text: str) -> List[float]:
    """
    Generate embeddings using Sentence Transformers (local, no API needed)
    """
    try:
        embedding = model.encode(text, convert_to_numpy=True)
        return embedding.tolist()
    except Exception as e:
        print(f"❌ Error generating embedding: {e}")
        raise

def process_and_upload_documents(urls: List[str]):
    """
    Process all URLs and upload embeddings to Qdrant
    """
    all_points = []
    point_id = 1

    for idx, url in enumerate(urls, 1):
        print(f"\n[{idx}/{len(urls)}] Processing: {url}")

        # Extract text
        text = extract_text_from_url(url)
        if not text:
            print("  ⚠️  Skipped (no content)")
            continue

        # Chunk text
        chunks = chunk_text(text)
        print(f"  📄 {len(chunks)} chunks")

        # Generate embeddings for each chunk
        for chunk_idx, chunk in enumerate(chunks):
            try:
                # Generate embedding (runs locally - super fast!)
                embedding = generate_embeddings(chunk)

                # Create point
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "url": url,
                        "content": chunk,
                        "chunk_index": chunk_idx,
                        "total_chunks": len(chunks),
                        "source": "sitemap"
                    }
                )
                all_points.append(point)
                point_id += 1

                # Show progress for large documents
                if len(chunks) > 5 and (chunk_idx + 1) % 5 == 0:
                    print(f"    Processing chunk {chunk_idx + 1}/{len(chunks)}...")

            except Exception as e:
                print(f"  ❌ Error processing chunk {chunk_idx + 1}: {e}")
                continue

        print(f"  ✅ Completed ({len(chunks)} chunks)")

    # Upload all points in batches
    if all_points:
        batch_size = 100
        total_batches = (len(all_points) + batch_size - 1) // batch_size

        print(f"\n📤 Uploading {len(all_points)} embeddings in {total_batches} batches...")

        for i in range(0, len(all_points), batch_size):
            batch = all_points[i:i + batch_size]
            batch_num = (i // batch_size) + 1

            try:
                qdrant.upsert(
                    collection_name=COLLECTION_NAME,
                    points=batch
                )
                print(f"  ✓ Batch {batch_num}/{total_batches} uploaded")
            except Exception as e:
                print(f"  ❌ Error uploading batch {batch_num}: {e}")

        # Verify
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print(f"\n✅ Verification: {collection_info.points_count} points in collection")
    else:
        print("\n⚠️  No embeddings were created")

def main():
    """
    Main execution function
    """
    print("=" * 60)
    print("🚀 Humanoid Robotics Embedding Generation")
    print("=" * 60)
    print(f"\n📊 Configuration:")
    print(f"  Sitemap: {SITEMAP_URL}")
    print(f"  Collection: {COLLECTION_NAME}")
    print(f"  Model: all-MiniLM-L6-v2 (Sentence Transformers)")
    print(f"  Vector Size: {EMBED_DIM} dimensions")
    print(f"  Mode: LOCAL (No API needed - 100% FREE!)")
    print(f"  Qdrant: {os.getenv('QDRANT_URL', 'Not set')[:50]}...")

    try:
        # Step 1: Get all URLs from sitemap
        print("\n" + "=" * 60)
        print("STEP 1: Fetching URLs from sitemap")
        print("=" * 60)
        urls = get_all_urls(SITEMAP_URL)

        if not urls:
            print("❌ No URLs found. Exiting.")
            return

        # Step 2: Create collection
        print("\n" + "=" * 60)
        print("STEP 2: Creating Qdrant collection")
        print("=" * 60)
        create_collection()

        # Step 3: Process and upload
        print("\n" + "=" * 60)
        print("STEP 3: Processing documents and generating embeddings")
        print("=" * 60)
        process_and_upload_documents(urls)

        print("\n" + "=" * 60)
        print("🎉 Embedding Generation Complete!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n⚠️  Process interrupted by user")
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        raise

if __name__ == "__main__":
    main()
