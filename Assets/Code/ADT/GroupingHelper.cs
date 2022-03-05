using System;
using Unity.Collections;
using UnityEngine.Assertions;

namespace Balma.ADT
{
	public struct GroupingHelper : IDisposable
	{
		public const int NO_GROUP = -1;

		private NativeList<int> _groupToIsland;
		private int _nextIsland;

		public GroupingHelper(Allocator allocator)
		{
			_nextIsland = 0;
			_groupToIsland = new NativeList<int>(allocator);
		}

		public void Dispose()
		{
			_groupToIsland.Dispose();
		}

		public int GetIsland(int group) => group < 0 ? -1 : _groupToIsland[group];

		public bool AreInSameIsland(int g1, int g2)
		{
			if (g1 == NO_GROUP || g2 == NO_GROUP)
				return false;
			else
				return _groupToIsland[g1] == _groupToIsland[g2];
		}

		// Join islands for groups.
		public void JoinGroups(ref int g1, ref int g2)
		{
			if (g1 == NO_GROUP && g2 == NO_GROUP)
				g1 = g2 = NewGroup(); // No groups, use a new one (with a new island).
			else if (g1 == NO_GROUP)
				g1 = g2; // Use the other group
			else if (g2 == NO_GROUP)
				g2 = g1; // Use the other group
			else if (g1 != g2 && _groupToIsland[g1] != _groupToIsland[g2])
			{
				// Different groups & different islands, we need to join islands. Choose lowest number to coalesce towards low-end.
				if (_groupToIsland[g1] > _groupToIsland[g2])
					_groupToIsland[g1] = _groupToIsland[g2];
				else
					_groupToIsland[g2] = _groupToIsland[g1];
			}
		}

		private int NewGroup()
		{
			_groupToIsland.Add(_nextIsland++);
			return _groupToIsland.Length - 1;
		}
	}
}