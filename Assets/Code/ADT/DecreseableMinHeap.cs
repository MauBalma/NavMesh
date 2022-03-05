using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace Balma.ADT
{
	public struct MinHeap<T> : IDisposable where T : unmanaged, IEquatable<T>
	{
		private NativeList<T> items;
		private NativeList<float> values;
		private NativeHashMap<T, int> map;
		
		public int Count => items.Length;
		public T Peek() => items[0];

		public MinHeap(Allocator allocator)
		{
			items = new NativeList<T>(allocator);
			values = new NativeList<float>(allocator);
			map = new NativeHashMap<T, int>(1024, allocator);
		}
		
		public void Dispose()
		{
			items.Dispose();
			values.Dispose();
			map.Dispose();
		}

		public void Clear()
		{
			items.Clear();
			values.Clear();
			map.Clear();
		}
		
		public void Push(T item, float value)
		{
			// If the item already existed, update and exit
			if (DecreaseKey(item, value))
				return;

			// Add last (a leaf)
			items.Add(item);
			values.Add(value);

			var current = Count - 1;
			map[item] = current;

			// Keep climbing to top until parent or unable to improve heap
			while (current != 0)
			{
				var parent = (current - 1) / 2; //Integer division (round down)
				if (value < values[parent])
				{
					Swap(current, parent);
					current = parent;
				}
				else
					break;
			}
		}
		
		public T Pop(out float value)
		{
			var result = items[0];
			value = values[0];

			//Remove first, swap with last and decrease count
			items[0] = items[Count - 1];
			values[0] = values[Count - 1];
			values.RemoveAt(Count - 1);
			items.RemoveAt(Count - 1);

			if (Count > 0)
			{
				map[items[0]] = 0;

				// Trickle first down
				int current = 0;
				while (current < Count)
				{
					int child0 = current * 2 + 1;
					int child1 = current * 2 + 2;
					int best = current;
					if (child0 < Count && values[child0] < values[best])
						best = child0;
					if (child1 < Count && values[child1] < values[best])
						best = child1;

					if (best == current)
						break; // Can't improve
					else
					{
						Swap(current, best);
						current = best; // Go down that way
					}
				}
			}

			map.Remove(result);
			return result;
		}

		public T Pop()
		{
			return Pop(out var v);
		}

		private void Swap(int i, int j)
		{
			var tt = items[i];
			items[i] = items[j];
			items[j] = tt;

			var tv = values[i];
			values[i] = values[j];
			values[j] = tv;

			map[items[i]] = i;
			map[items[j]] = j;

		}

		// Returns true if it found the item.
		private bool DecreaseKey(T item, float newValue)
		{
			if (!map.TryGetValue(item, out var current))
				return false;

			var oldValue = values[current];
			if (oldValue > newValue)
			{
				values[current] = newValue;

				// Bubble up
				while (current != 0)
				{
					var parent = (current - 1) / 2; //Integer division (round down)
					if (newValue < values[parent])
					{
						Swap(current, parent);
						current = parent;
					}
					else break;
				}
			}

			return true;
		}
	}
}