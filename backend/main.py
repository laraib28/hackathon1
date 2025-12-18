import os
import requests
import xml.etree.ElementTree as ET
import trafilatura
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from sentence_transformers import SentenceTransformer

# ===============================
# Load ENV
# ===============================
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

SITEMAP_URL = "https://hackathon1-r5lc.vercel.app/sitemap.xml"

if not QDRANT_URL:
    raise ValueError("❌ QDRANT_URL missing in .env")

# ===============================
# Init embedding model (LOCAL – FREE)
# ===============================
print("🔄 Loading embedding model...")
model = SentenceTransformer("all-MiniLM-L6-v2")
VECTOR_SIZE = 384
print("✅ Model loaded")

# ===============================
# Init Qdrant
# ===============================
print("🔄 Connecting to Qdrant...")
qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)
print("✅ Qdrant connected")

# ===============================
# Helpers
# ===============================
def get_urls_from_sitemap():
    print("🔄 Fetching sitemap...")
    res = requests.get(SITEMAP_URL)
    root = ET.fromstring(res.text)

    ns = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}
    urls = []

    for url in root.findall(".//ns:url", ns):
        loc = url.find("ns:loc", ns)
        if loc is not None:
            urls.append(loc.text)

    print(f"✅ {len(urls)} URLs found")
    return urls


def chunk_text(text, size=500):
    words = text.split()
    chunks = []
    for i in range(0, len(words), size):
        chunk = " ".join(words[i:i+size])
        if len(chunk) > 100:
            chunks.append(chunk)
    return chunks


# ===============================
# Create collection
# ===============================
def create_collection():
    try:
        qdrant.delete_collection(COLLECTION_NAME)
        print("🗑️ Old collection deleted")
    except:
        pass

    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=VECTOR_SIZE,
            distance=Distance.COSINE
        )
    )
    print(f"✅ Collection created: {COLLECTION_NAME}")


# ===============================
# Main process
# ===============================
def main():
    urls = get_urls_from_sitemap()
    create_collection()

    points = []
    pid = 1

    for url in urls:
        print(f"\n📄 Processing: {url}")
        downloaded = trafilatura.fetch_url(url)
        text = trafilatura.extract(downloaded)

        if not text:
            print("⚠️ No text, skipped")
            continue

        chunks = chunk_text(text)
        print(f"🔹 {len(chunks)} chunks")

        embeddings = model.encode(chunks)

        for chunk, vector in zip(chunks, embeddings):
            points.append(
                PointStruct(
                    id=pid,
                    vector=vector.tolist(),
                    payload={
                        "url": url,
                        "content": chunk
                    }
                )
            )
            pid += 1

    if points:
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"\n🎉 DONE! {len(points)} embeddings uploaded")
    else:
        print("❌ No embeddings created")


if __name__ == "__main__":
    main()
