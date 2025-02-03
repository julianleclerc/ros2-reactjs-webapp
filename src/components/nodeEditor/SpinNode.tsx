import React, { memo, type ReactNode } from 'react';
import { Handle, Position, type Node, type NodeProps } from '@xyflow/react';

export type SpinNodeData = {
  title: string;
  icon?: ReactNode;
  subline?: string;
};

export default memo(({ data }: NodeProps<Node<SpinNodeData>>) => {
  return (
    <>
      <div className={`wrapper ${data.state ? data.state : ""}`}>
        <div className="inner">
          <div className="body">
            {data.icon && <div className="icon">{data.icon}</div>}
            <div>
              <div className="title">{data.title}</div>
              {data.subline && <div className="subline">{data.subline}</div>}
            </div>
          </div>
          <Handle type="target" position={Position.Top} />
          <Handle type="source" position={Position.Bottom} />
        </div>
      </div>
    </>
  );
});
