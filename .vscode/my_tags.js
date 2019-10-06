
exports.execute = async (args) => {
    const vscode = args.require('vscode');
    const editor = vscode.window.activeTextEditor;
    let insertPosition = editor.selection.active;
    let list = ['zmq', 'protobuf', 'sdf', 'plugin'].sort();
    let i = 0;
	const result = await vscode.window.showQuickPick(list).then(
        selection=>{
            if (!selection) {
                return;
              }
            editor.edit(edit => 
            {
                edit.insert(insertPosition, selection);
            })
        }
    );
    //     canPickMany: true,
    //     // placeHolder: 'eins, zwei or drei',
    //     // onDidSelectItem: item => vscode.window.showInformationMessage(`Focus ${++i}: ${item}`)
    //     onDidSelectItem: item =>editor.edit(edit => 
    //         {
    //         edit.insert(insertPosition, `${item}`);
    //         })
	// });
    // vscode.window.showInformationMessage(`Got: ${result}`);
    // const result = await vscode.window.showInputBox({
	// 	value: 'abcdef',
	// 	valueSelection: [2, 4],
	// 	placeHolder: 'For example: fedcba. But not: 123',
	// 	validateInput: text => {
	// 		vscode.window.showInformationMessage(`Validating: ${text}`);
	// 		return text === '123' ? 'Not 123!' : null;
	// 	}
	// });
	// vscode.window.showInformationMessage(`Got: ${result}`);
};