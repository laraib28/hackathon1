# Embedding Setup Guide

This guide explains how to populate and manage your Qdrant vector database with document embeddings.

## Overview

The system uses semantic search powered by:
- **Qdrant**: Vector database for storing embeddings
- **Embedding Providers**: Cohere API (preferred) or SentenceTransformers (local fallback)
- **Documents**: Markdown files from the humanoid robotics book

## Scripts

### 1. `populate_embeddings.py` - Main Population Script

Comprehensive script that:
1. Loads all markdown files from `../humanoid-robotics-book/docs`
2. Chunks documents into manageable pieces (500 chars with 100 char overlap)
3. Cleans markdown syntax for better embeddings
4. Generates embeddings using Cohere or SentenceTransformers
5. Upserts embeddings into Qdrant collection
6. Reports statistics

**Usage:**
```bash
# Using virtual environment
source .venv/bin/activate
python populate_embeddings.py

# Or using uv
uv run populate_embeddings.py
```

**What it does:**
- Creates the `humanoid_robotics_docs` collection if needed
- Processes ~31 markdown files into chunks
- Generates embeddings for each chunk
- Stores in Qdrant with metadata (URL, chunk index, source file)

**First Run Note:**
- First run may take 2-5 minutes as it downloads the SentenceTransformer model (~80-400MB)
- Subsequent runs are much faster

### 2. `check_embeddings.py` - Quick Status Check

Simple script to verify if embeddings exist in your collection.

**Usage:**
```bash
source .venv/bin/activate
python check_embeddings.py

# Or
uv run check_embeddings.py
```

**Output:**
```
✅ Collection: humanoid_robotics_docs
📊 Total vectors: 156
📐 Vector size: 384
📍 Distance: COSINE
🟢 Status: green
```

### 3. `embedding_service.py` - Embedding Provider

Unified interface for different embedding providers.

**Providers:**
- **Cohere** (1024 dimensions): Requires `COHERE_API_KEY` in `.env`
- **SentenceTransformers** (384 dimensions): Local, free, no API key needed

**Auto-detection:**
- If `COHERE_API_KEY` is set, uses Cohere
- Otherwise falls back to SentenceTransformers

### 4. `search.py` - Search Interface

Provides semantic search functionality.

**Usage:**
```bash
python search.py
```

Interactive search mode:
```
🔍 Semantic Search - Interactive Mode
Enter your search query (or 'quit' to exit):
>>> what is ROS2?

✅ Found 5 results:
[1] Score: 0.85
    URL: /docs/part2-modules/module1-ros2/chapter-04-introduction-to-ros2.md
    Content: ROS2 (Robot Operating System 2) is a flexible framework...
```

## Configuration

### Environment Variables (.env)

```bash
# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=humanoid_robotics_docs

# Optional: Cohere API (for better embeddings)
COHERE_API_KEY=your-cohere-api-key

# Database (for user tracking in search API)
DATABASE_URL=postgresql://...
```

### Adjustable Parameters

In `populate_embeddings.py`:

```python
COLLECTION_NAME = "humanoid_robotics_docs"  # Collection name
DOCS_FOLDER = "../humanoid-robotics-book/docs"  # Source folder
CHUNK_SIZE = 500  # Characters per chunk
CHUNK_OVERLAP = 100  # Overlap between chunks
BATCH_SIZE = 10  # Embeddings per batch
```

## Workflow

### Initial Setup

1. **Install dependencies:**
   ```bash
   uv sync
   ```

2. **Configure environment:**
   ```bash
   cp .env.example .env
   # Edit .env with your credentials
   ```

3. **Populate embeddings:**
   ```bash
   uv run populate_embeddings.py
   ```

4. **Verify:**
   ```bash
   uv run check_embeddings.py
   ```

5. **Test search:**
   ```bash
   uv run search.py
   ```

### Updating Documents

When you add or modify markdown files:

```bash
# Re-run population (it will upsert, replacing existing chunks)
uv run populate_embeddings.py
```

### Troubleshooting

**Problem:** `No module named 'qdrant_client'`
**Solution:** Install dependencies with `uv sync` or `pip install -r requirements.txt`

**Problem:** `Collection doesn't exist`
**Solution:** Run `populate_embeddings.py` to create it

**Problem:** `COHERE_API_KEY not found`
**Solution:** This is OK! Script will fall back to SentenceTransformers

**Problem:** First run is very slow
**Solution:** Normal! SentenceTransformers downloads model on first run

**Problem:** Search returns no results
**Solution:** Lower `score_threshold` parameter or check if embeddings exist

## Architecture

```
┌─────────────────┐
│ Markdown Files  │
│ (.md in docs/)  │
└────────┬────────┘
         │
         ├─ Load & Parse
         │
         ├─ Clean Markdown
         │
         ├─ Chunk (500 chars)
         │
         ▼
┌────────────────────┐
│ Document Chunks    │
└────────┬───────────┘
         │
         ├─ Generate Embeddings
         │  (Cohere or ST)
         │
         ▼
┌────────────────────┐
│  Qdrant Vector DB  │
│  - vectors         │
│  - metadata        │
│  - URLs            │
└────────┬───────────┘
         │
         ├─ Semantic Search
         │
         ▼
┌────────────────────┐
│  Search Results    │
│  with scores       │
└────────────────────┘
```

## API Integration

The search functionality is exposed via FastAPI endpoints in `qdrant_api.py`:

### Endpoints

**POST /api/search**
```json
{
  "query": "what is ROS2?",
  "user_id": "optional-user-id",
  "limit": 5,
  "score_threshold": 0.7
}
```

**GET /api/qdrant/stats**
Returns collection statistics

**GET /api/user/search-history/{user_id}**
Returns user's search history

## Performance

### Embedding Generation Speed

- **Cohere API**: ~1-2 seconds per batch (10 chunks)
- **SentenceTransformers**: ~2-5 seconds per batch (10 chunks)

### Search Speed

- **Query time**: ~50-200ms per search
- **Results**: Returns top 5 results by default

### Storage

- **Each chunk**: ~384-1024 dimensions (1.5-4 KB per vector)
- **31 markdown files**: ~100-200 chunks
- **Total storage**: ~150-800 KB

## Best Practices

1. **Chunk Size**: Keep chunks 300-1000 characters for best results
2. **Overlap**: Use 50-150 character overlap to avoid context loss
3. **Batch Size**: Process 10-50 embeddings per batch
4. **Score Threshold**: Use 0.5-0.7 for general queries, 0.8+ for exact matches
5. **Regular Updates**: Re-run population when docs change significantly

## Resources

- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Cohere Embeddings](https://cohere.com/embeddings)
- [SentenceTransformers](https://www.sbert.net/)
